/******************************************************************************
 * Copyright (c) 2021
 * KELO Robotics GmbH
 *
 * Author:
 * Walter Nowak
 * Sebastian Blumenthal
 * Dharmin Bakaraniya
 * Nico Huebel
 * Arthur Ketels
 *
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of Locomotec nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************/


#include "kelo_tulip/PlatformDriverROS.h"
#include "kelo_tulip/OdometryFreshness.h"
#include "kelo_tulip/ParameterValidation.h"
#include "kelo_tulip/VelocityCommand.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace kelo {

PlatformDriverROS::PlatformDriverROS()
	: driver(NULL)
	, odom_broadcaster(nullptr)
{
	nWheels = 0;

	debugMode = false;
	activeByJoypad = false;

	// On a robot where a separate state estimator owns odom -> base_footprint,
	// this driver must not publish that transform too; enable it only when
	// running the driver standalone or for bench testing.
	publishTf = false;
	odomFrame = "odom";
	baseFrame = "base_footprint";

	odomx = 0;
	odomy = 0;
	odoma = 0;
	staleTwistCovariance = 1e6;
	encoderDeltaLimit = 0.1;
	maxHeldGap = 0.1;
}

PlatformDriverROS::~PlatformDriverROS() {
	if (driver)
		delete driver;

	if (odom_broadcaster)
		odom_broadcaster.reset();
		//delete odom_broadcaster;
}

bool PlatformDriverROS::init(rclcpp::Node::SharedPtr nh, std::string configPrefix) {
	this->nh = nh;
	const DriverLimits defaults;
	
	nh->declare_parameter("num_wheels", 0);
	nh->declare_parameter("vlin_max", defaults.vlinMax);
	nh->declare_parameter("va_max", defaults.vaMax);
	nh->declare_parameter("vlin_acc_max", defaults.vlinAccMax);
	nh->declare_parameter("vlin_dec_max", defaults.vlinDecMax);
	nh->declare_parameter("va_acc_max", defaults.vaAccMax);
	nh->declare_parameter("va_dec_max", defaults.vaDecMax);
	nh->declare_parameter("angle_acc_max", defaults.angleAccMax);
	// The ramp the driver runs on its own when it is stopped while moving.
	nh->declare_parameter("stop_vlin_dec", defaults.stopVlinDec);
	nh->declare_parameter("stop_va_dec", defaults.stopVaDec);
	nh->declare_parameter("stop_timeout", defaults.stopTimeoutSec);
	nh->declare_parameter("active_by_joypad", false);
	nh->declare_parameter("odom_frame", odomFrame);
	nh->declare_parameter("base_frame", baseFrame);
	nh->declare_parameter("publish_tf", publishTf);
	nh->declare_parameter("cmd_vel_timeout", cmdVelWatchdog.getTimeout());
	nh->declare_parameter("odom_stale_ticks", OdometryFreshnessTracker::DEFAULT_STALE_TICKS);
	nh->declare_parameter("stale_twist_covariance", staleTwistCovariance);

	// Dynamic typing so an integer in the yaml (30 for 30.0) is read, not thrown on.
	const CurrentShapingConfig shapingDefaults;
	rcl_interfaces::msg::ParameterDescriptor anyNumber;
	anyNumber.dynamic_typing = true;
	const std::pair<const char *, float> shapingParams[] = {
		{"wheel_slew_rate", shapingDefaults.slewRateRadPerSecSq},
		{"pivot_kp", shapingDefaults.pivotKp},
		{"pivot_max_error", shapingDefaults.maxPivotError},
		{"pivot_max_correction_speed", shapingDefaults.maxPivotCorrectionSpeed},
		{"reorient_start_error", shapingDefaults.reorientStartError},
		{"reorient_full_error", shapingDefaults.reorientFullError},
		{"reorient_min_scale", shapingDefaults.reorientMinScale},
	};
	for (const auto &param : shapingParams)
		nh->declare_parameter(param.first, rclcpp::ParameterValue((double)param.second), anyNumber);

	rclcpp::Parameter num_wheels;
	if (!nh->get_parameter("num_wheels", num_wheels)) {
		RCLCPP_FATAL(nh->get_logger(), "Missing number of wheels in config file");
		return false;
	}
	nWheels = num_wheels.as_int();

	if (nWheels < 1 || nWheels > bounds::MAX_WHEELS) {
		RCLCPP_FATAL(nh->get_logger(), "num_wheels = %d is outside [1, %d]; refusing to start", nWheels,
			bounds::MAX_WHEELS);
		return false;
	}

	wheelConfigs.resize(nWheels);
	kelo::WheelData data = {};
	data.enable = true;
	data.error = false;
	data.errorTimestamp = false;
	wheelData.resize(nWheels, data);

	// read all wheel configs
	configErrors.clear();
	readWheelModels();
	readWheelConfig();

	// Anything but an int or a double reads as NaN, which isValid rejects.
	auto shapingNumber = [&nh](const char *name) {
		const rclcpp::Parameter param = nh->get_parameter(name);
		switch (param.get_type()) {
			case rclcpp::ParameterType::PARAMETER_DOUBLE: return param.as_double();
			case rclcpp::ParameterType::PARAMETER_INTEGER: return (double)param.as_int();
			default: return std::numeric_limits<double>::quiet_NaN();
		}
	};
	CurrentShapingConfig shaping;
	shaping.slewRateRadPerSecSq = shapingNumber("wheel_slew_rate");
	shaping.pivotKp = shapingNumber("pivot_kp");
	shaping.maxPivotError = shapingNumber("pivot_max_error");
	shaping.maxPivotCorrectionSpeed = shapingNumber("pivot_max_correction_speed");
	shaping.reorientStartError = shapingNumber("reorient_start_error");
	shaping.reorientFullError = shapingNumber("reorient_full_error");
	shaping.reorientMinScale = shapingNumber("reorient_min_scale");

	DriverLimits limits;
	limits.vlinMax = nh->get_parameter("vlin_max").as_double();
	limits.vaMax = nh->get_parameter("va_max").as_double();
	limits.vlinAccMax = nh->get_parameter("vlin_acc_max").as_double();
	limits.vlinDecMax = nh->get_parameter("vlin_dec_max").as_double();
	limits.vaAccMax = nh->get_parameter("va_acc_max").as_double();
	limits.vaDecMax = nh->get_parameter("va_dec_max").as_double();
	limits.angleAccMax = nh->get_parameter("angle_acc_max").as_double();
	limits.stopVlinDec = nh->get_parameter("stop_vlin_dec").as_double();
	limits.stopVaDec = nh->get_parameter("stop_va_dec").as_double();
	limits.stopTimeoutSec = nh->get_parameter("stop_timeout").as_double();
	// If the /cmd_vel publisher dies or stalls, the last command would
	// otherwise be held forever. There is deliberately no way to disable this.
	limits.cmdVelTimeoutSec = nh->get_parameter("cmd_vel_timeout").as_double();

	const int staleTicks = nh->get_parameter("odom_stale_ticks").as_int();
	staleTwistCovariance = nh->get_parameter("stale_twist_covariance").as_double();

	// A driver that moves a robot starts on a valid configuration or not at all.
	std::vector<std::string> errors = configErrors;
	for (const std::string &error : wheelConfigErrors(wheelConfigs))
		errors.push_back(error);
	for (const std::string &error : limitErrors(limits))
		errors.push_back(error);
	std::string shapingProblem;
	if (!isValid(shaping, &shapingProblem))
		errors.push_back("current shaping: " + shapingProblem);
	if (staleTicks < 1)
		errors.push_back("odom_stale_ticks = " + std::to_string(staleTicks) + " is below 1");
	if (!isValidStaleTwistCovariance(staleTwistCovariance))
		errors.push_back("stale_twist_covariance = " + std::to_string(staleTwistCovariance) +
			" must be finite and > 0");
	if (!errors.empty()) {
		for (const std::string &error : errors)
			RCLCPP_FATAL(nh->get_logger(), "Invalid parameter: %s", error.c_str());
		RCLCPP_FATAL(nh->get_logger(), "Refusing to start with %zu invalid parameter(s)", errors.size());
		return false;
	}

	driver = createDriver();
	driver->setCurrentShaping(shaping);
	driver->setMaxvlin(limits.vlinMax);
	driver->setMaxva(limits.vaMax);
	driver->setMaxvlinacc(limits.vlinAccMax);
	driver->setMaxvlindec(limits.vlinDecMax);
	driver->setMaxangleacc(limits.angleAccMax);
	driver->setMaxvaacc(limits.vaAccMax);
	driver->setMaxvadec(limits.vaDecMax);
	driver->setStopParameters(limits.stopVlinDec, limits.stopVaDec, limits.stopTimeoutSec);
	// The EtherCAT cycle enforces this; the ROS loop only reports it.
	driver->setCommandTimeout(limits.cmdVelTimeoutSec);
	cmdVelWatchdog.setTimeout(limits.cmdVelTimeoutSec);

	rclcpp::Parameter b;
	if (nh->get_parameter("active_by_joypad", b))
		activeByJoypad = b.as_bool();
	if (!activeByJoypad)
		driver->setCanChangeActive();

	rclcpp::Parameter frameParam;
	if (nh->get_parameter("odom_frame", frameParam))
		odomFrame = frameParam.as_string();
	if (nh->get_parameter("base_frame", frameParam))
		baseFrame = frameParam.as_string();
	rclcpp::Parameter publishTfParam;
	if (nh->get_parameter("publish_tf", publishTfParam))
		publishTf = publishTfParam.as_bool();

	encoderDeltaLimit = encoderDeltaLimitSec(staleTicks, LOOP_PERIOD_SEC, wheelAliasingLimitSec());
	maxHeldGap = staleTicks * LOOP_PERIOD_SEC + 0.5 * LOOP_PERIOD_SEC;
	freshnessTracker = std::make_unique<OdometryFreshnessTracker>(nWheels, staleTicks);

	odomPublisher = nh->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
	odomInitializedPublisher = nh->create_publisher<std_msgs::msg::Empty>("/odom_initialized", 10);
//	timestampPublisher = nh->create_publisher<std_msgs::msg::UInt64MultiArray>("timestamp", 10);
	imuPublisher = nh->create_publisher<sensor_msgs::msg::Imu>("~/imu", 10);
	processDataInputPublisher = nh->create_publisher<kelo_tulip::msg::KeloDrivesInput>("~/wheels_input", 10);
	batteryPublisher = nh->create_publisher<std_msgs::msg::Float32>("~/battery", 10);
	errorPublisher = nh->create_publisher<std_msgs::msg::Int32>("~/error", 10);
	statusPublisher = nh->create_publisher<std_msgs::msg::Int32>("~/status", 10);
	joySubscriber = nh->create_subscription<sensor_msgs::msg::Joy>("/joy", 5, std::bind(&PlatformDriverROS::joyCallback, this, std::placeholders::_1));
	cmdVelSubscriber = nh->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, std::bind(&PlatformDriverROS::cmdVelCallback, this, std::placeholders::_1));
	resetSubscriber = nh->create_subscription<std_msgs::msg::Empty>("reset", 1, std::bind(&PlatformDriverROS::resetCallback, this, std::placeholders::_1));
	enableSubscriber = nh->create_subscription<std_msgs::msg::Int32MultiArray>("wheels_enable", 10, std::bind(&PlatformDriverROS::enableCallback, this, std::placeholders::_1));
	
	odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(nh);
	
	initializeEncoderValue();
	
	return true;
}

bool PlatformDriverROS::step() {
	// The EtherCAT cycle already ramps a silent /cmd_vel to zero on its own;
	// this is the second line and the log.
	if (cmdVelWatchdog.checkExpired(CommandWatchdog::Clock::now())) {
		driver->setTargetVelocity(0, 0, 0);
		RCLCPP_WARN(nh->get_logger(), "No /cmd_vel for %.3f s, commanding zero velocity", cmdVelWatchdog.getTimeout());
	}

	checkAndPublishSmartWheelStatus();

	//calculate robot velocity
	double vx, vy, va, displacement, dt;
	//calculateRobotVelocity(vx, vy, va, displacement);
	OdometryFreshnessTracker::Update freshness = calculateRobotVelocity2(vx, vy, va, displacement, dt);
	// Without fresh wheel data the velocities are zero and the pose does not
	// move: reporting the frozen last motion would drift the state estimator.
	if (freshness.noDataWarning)
		RCLCPP_ERROR(nh->get_logger(), "No wheel data received since start, odometry stays at zero");
	if (freshness.becameStale)
		RCLCPP_ERROR(nh->get_logger(), "Wheel data is stale, publishing zero odometry velocity");
	else if (freshness.becameFresh)
		RCLCPP_WARN(nh->get_logger(), "Wheel data is fresh again, odometry resumes");

	//calculate robot displacement and current pose
	//calculateRobotPose(vx, vy, va);
	calculateRobotPose2(vx, vy, va, dt);

		
	//publish the odometry
	publishOdometry(vx, vy, va, holdOdometry(freshness.stale, freshness.noNewSample));

	// publish_tf defaults false so the driver never competes with a separate
	// state estimator for odom -> base_footprint. Enable it only for bench
	// testing or standalone use.
	if (publishTf) {
		geometry_msgs::msg::TransformStamped odom_trans;
		createOdomToBaseLinkTransform(odom_trans);
		odom_broadcaster->sendTransform(odom_trans);
	}

/*
		//publish smartwheel values
		std_msgs::msg::float64_multi_array processDataValues;
		for (unsigned int i = 0; i < wheelConfigs.size(); i++) {
			addToWheelDataMsg(processDataValues, driver->getWheelData(i));
			addToProcessDataMsg(processDataValues, driver->getProcessData(wheelConfigs[i].ethercatNumber));
			processDataValues.data.push_back(driver->getCurrentDrive());
			processDataValues.data.push_back(driver->getThreadPhase());
		}
		valuesPublisher->publish(processDataValues);
*/

	publishProcessDataInput();
	publishBattery();

	//publish IMU data
	publishIMU();

	return true;
}

std::string PlatformDriverROS::getType() {
	return "platform_driver";
}

EtherCATModule* PlatformDriverROS::getEtherCATModule() {
	return driver;
}

kelo::PlatformDriver* PlatformDriverROS::createDriver() {
	return new kelo::PlatformDriver(wheelConfigs, wheelData);
}

void PlatformDriverROS::readWheelModels() {
	nh->declare_parameter("wheel_models.list", std::vector<std::string>{});
	rclcpp::Parameter list = nh->get_parameter("wheel_models.list");
	std::vector<std::string> parameterList = list.as_string_array();

	for (unsigned int i = 0; i < parameterList.size(); i++) {
		std::string name = parameterList[i];
		std::string prefix = "wheel_models." + name + ".";
		nh->declare_parameter(prefix + "active", true);
		nh->declare_parameter(prefix + "diameter", 0.105);
		nh->declare_parameter(prefix + "width", 0.040);
		nh->declare_parameter(prefix + "casteroffset", 0.010);
		nh->declare_parameter(prefix + "wheeldistance", 0.08);
		nh->declare_parameter(prefix + "canPivot", true);
		nh->declare_parameter(prefix + "velocitylimit", 100.0);
		nh->declare_parameter(prefix + "currentlimit", 10.0);
		nh->declare_parameter(prefix + "standbycurrent", 1.0);

		WheelModel wm;
		wm.name = name;
		wm.active = nh->get_parameter(prefix + "active").as_bool();
		wm.diameter = nh->get_parameter(prefix + "diameter").as_double();
		wm.width = nh->get_parameter(prefix + "width").as_double();
		wm.casteroffset = nh->get_parameter(prefix + "casteroffset").as_double();
		wm.wheeldistance = nh->get_parameter(prefix + "wheeldistance").as_double();
		wm.canPivot = nh->get_parameter(prefix + "canPivot").as_bool();
		wm.velocitylimit = nh->get_parameter(prefix + "velocitylimit").as_double();
		wm.currentlimit = nh->get_parameter(prefix + "currentlimit").as_double();
		wm.standbycurrent = nh->get_parameter(prefix + "standbycurrent").as_double();
		for (const std::string &error : wheelModelErrors(wm))
			configErrors.push_back(error);
		wheelModels[name] = wm;
	}
	
	//XmlRpc::XmlRpcValue xmllist;
	//nh.getParam("wheel_models", xmllist);
	//for (XmlRpc::XmlRpcValue::iterator it = xmllist.begin(); it != xmllist.end(); ++it) {
		//std::string name = it->first;
		//std::string prefix = "wheel_models/" + name + "/";
		//WheelModel wm;
		//wm.name = name;
		//nh.getParam(prefix + "active", wm.active);
		//nh.getParam(prefix + "diameter", wm.diameter);
		//nh.getParam(prefix + "width", wm.width);
		//nh.getParam(prefix + "casteroffset", wm.casteroffset);
		//nh.getParam(prefix + "wheeldistance", wm.wheeldistance);
		//nh.getParam(prefix + "can_pivot", wm.canPivot);
		//nh.getParam(prefix + "velocitylimit", wm.velocitylimit);
		//nh.getParam(prefix + "currentlimit", wm.currentlimit);
		//wheelModels[name] = wm;
	//}
}

void PlatformDriverROS::readWheelConfig() {
	for (int i = 0; i < nWheels; i++) {
		std::stringstream ssGroupName;
		ssGroupName << "wheel" << i;
		std::string groupName = ssGroupName.str();
		nh->declare_parameter(groupName + ".ethercat_number", 0);
		nh->declare_parameter(groupName + ".x", 0.0);
		nh->declare_parameter(groupName + ".y", 0.0);
		nh->declare_parameter(groupName + ".a", 0.0);
		nh->declare_parameter(groupName + ".model", "KD100");

		kelo::WheelConfig config;
		config.enable = true;
		config.reverseVelocity = true;
		rclcpp::Parameter ecatNr, wheelx, wheely, wheela;
		bool ok =		
		     nh->get_parameter(groupName + ".ethercat_number", ecatNr)
		  && nh->get_parameter(groupName + ".x", wheelx)
			&& nh->get_parameter(groupName + ".y", wheely)
			&& nh->get_parameter(groupName + ".a", wheela);
		config.ethercatNumber = ecatNr.as_int();
		config.x = wheelx.as_double();
		config.y = wheely.as_double();
		config.a = wheela.as_double();

		rclcpp::Parameter reverseVelocity;
		if (nh->get_parameter(groupName + ".reverse_velocity", reverseVelocity))
			config.reverseVelocity = (reverseVelocity.as_int() != 0);

		if (!ok)
			RCLCPP_WARN(nh->get_logger(), "Missing config value for wheel %d", i);

		// copy complete model data if provided
		rclcpp::Parameter model;
		if (nh->get_parameter(groupName + ".model", model)) {
			if (wheelModels.count(model.as_string()) > 0) {
				config.model = wheelModels[model.as_string()];
			} else {
				configErrors.push_back(groupName + ".model = " + model.value_to_string() +
				" is not in wheel_models.list");
			}
		}

		// enable separate values for this wheel
		rclcpp::Parameter x;
		if (nh->get_parameter(groupName + ".wheel_distance", x))
			config.model.wheeldistance = x.as_double();
		if (nh->get_parameter(groupName + ".diameter", x))
			config.model.diameter = x.as_double();

		wheelConfigs[i] = config;
	}
}

void PlatformDriverROS::checkAndPublishSmartWheelStatus() {
	int status = driver->getDriverStatus();
	// int state = (status & 0x000000ff);
	int error = (status & 0xffffff00);
		
	std_msgs::msg::Int32 statusMsg;
	statusMsg.data = status;
	statusPublisher->publish(statusMsg);

	std_msgs::msg::Int32 errorMsg;
	if (error) {
		// TODO correct
		//stop navigation and start debug mode. Robot can only be moved with joystick
		debugMode = true;
		errorMsg.data = status;
		errorPublisher->publish(errorMsg);
		statusPublisher->publish(statusMsg);
	} else {
		if (debugMode) {
			debugMode = false;
			errorMsg.data = 0;
			errorPublisher->publish(errorMsg);
		}
	}
}

double norm(double x) {
	const double TWO_PI = 2.0 * M_PI;
	while (x < -M_PI) {
		x += TWO_PI;
	}
	while (x > M_PI) {
		x -= TWO_PI;
	}

	return x;
}

// A wheel's ground speed is at most the translation limit plus the rim speed
// of the yaw limit; the driver clips both, so these bound the encoder rate.
double PlatformDriverROS::wheelAliasingLimitSec() {
	if (nWheels == 0)
		return encoderAliasingLimitSec(0.0, 0.0);
	double vlinMax = nh->get_parameter("vlin_max").as_double();
	double vaMax = nh->get_parameter("va_max").as_double();
	double minRadius = std::numeric_limits<double>::max();
	double maxDistance = 0.0;
	for (int i = 0; i < nWheels; i++) {
		minRadius = std::min(minRadius, 0.5 * wheelConfigs[i].model.diameter);
		maxDistance = std::max(maxDistance, std::hypot(wheelConfigs[i].x, wheelConfigs[i].y));
	}
	return encoderAliasingLimitSec(minRadius, vlinMax + vaMax * maxDistance);
}

void PlatformDriverROS::initializeEncoderValue() {
	prev_left_enc.resize(nWheels, 0);
	prev_right_enc.resize(nWheels, 0);
	prev_pivot_enc.resize(nWheels, 0);

	for (int i=0; i<nWheels; i++) {
		std::vector<double> encoderValueInit = driver->getEncoderValue(i);
		prev_left_enc[i] = encoderValueInit[0];
		prev_right_enc[i] = encoderValueInit[1];
	}
}

void PlatformDriverROS::calculateRobotVelocity(double& vx, double& vy, double& va, double& displacement) {
	double dt = 0.05;
	
	//initialize the variables
	vx = 0;
	vy = 0;
	va = 0;
	displacement = 0;
	
	for (int i = 0; i < nWheels; i++) {
		double r_w = wheelConfigs[i].model.diameter / 2.0;
		double d_w = wheelConfigs[i].model.wheeldistance;
		double s_w = wheelConfigs[i].model.casteroffset;
		double s_d_ratio = s_w / d_w;

		txpdo1_t* swData = driver->getWheelProcessData(i);
		std::vector<double> encoderValue = driver->getEncoderValue(i);
		double wl = (encoderValue[0] - prev_left_enc[i]) / dt;
		double wr = -(encoderValue[1] - prev_right_enc[i]) / dt;
		displacement += 0.5 * wheelConfigs[i].model.diameter * (fabs(norm(encoderValue[0] - prev_left_enc[i])) + fabs(norm(encoderValue[1] - prev_right_enc[i])));
		prev_left_enc[i] = encoderValue[0];
		prev_right_enc[i] = encoderValue[1];
		double theta = norm(swData->encoder_pivot - wheelConfigs[i].a); // encoder_offset can be obtained from the yaml file or smartWheelDriver class

		if (!wheelConfigs[i].reverseVelocity) {
			vx += r_w * ((wl + wr) * cos(theta)); // + 2 * s_d_ratio * (wl - wr) * sin(theta));
			vy += r_w * ((wl + wr) * sin(theta)); // - 2 * s_d_ratio * (wl - wr) * cos(theta));
		} else {
			vx -= r_w * ((wl + wr) * cos(theta)); // + 2 * s_d_ratio * (wl - wr) * sin(theta));
			vy -= r_w * ((wl + wr) * sin(theta)); // - 2 * s_d_ratio * (wl - wr) * cos(theta));		
		}
		double wangle = atan2(wheelConfigs[i].y, wheelConfigs[i].x);
		double d = sqrt(wheelConfigs[i].x * wheelConfigs[i].x + wheelConfigs[i].y * wheelConfigs[i].y);
		if (!wheelConfigs[i].reverseVelocity) {
			va += r_w * (2 * (wr - wl) * s_d_ratio * cos(theta - wangle) + (wr + wl) * sin(theta - wangle)) / d;
		} else{
			va += r_w * (2 * (wr - wl) * s_d_ratio * cos(theta - wangle) - (wr + wl) * sin(theta - wangle)) / d;
		}			
		//va += r_w * (wr + wl) * sin(theta - wangle) / d;
		//va += 4*swData->gyro_y;
	}
	// averaging the wheel velocity
	int nHubWheels = 2 * nWheels;
	if (nWheels > 0) {
		vx = vx / nHubWheels;
		vy = vy / nHubWheels;
		va = va / nHubWheels;
	}
}

void PlatformDriverROS::calculateRobotPose(double vx, double vy, double va) {
	double dt = 0.05;
	double dx, dy;
	
	if (fabs(va) > 0.001) {
		double vlin = sqrt(vx * vx + vy * vy);
		double direction = atan2(vy, vx);
		double circleRadius = fabs(vlin / va);
		double sign = 1;
		if (va < 0)
			sign = -1;
		//displacement relative to direction of movement
		double dx_rel = circleRadius * sin (fabs(va) * dt);
		double dy_rel = sign * circleRadius * (1 - cos(fabs(va) * dt));

		//transform displacement to previous robot frame
		dx = dx_rel * cos(direction) - dy_rel * sin(direction);
		dy = dx_rel * sin(direction) + dy_rel * cos(direction);
	}
	else {
		dx = vx * dt;
		dy = vy * dt;
	}
	
	//transform displacement to odom frame
	odomx += dx * cos(odoma) - dy * sin(odoma);
	odomy += dx * sin(odoma) + dy * cos(odoma);
	odoma = norm(odoma + va * dt);
}

OdometryFreshnessTracker::Update PlatformDriverROS::calculateRobotVelocity2(double& vx, double& vy, double& va, double& displacement, double &dt) {
	dt = LOOP_PERIOD_SEC; // Target delta time. Replaced by real delta from sensor timestamps, when available.
	std::vector<double> rx;
	rx.resize(nWheels, 0);
	std::vector<double> ry;
	ry.resize(nWheels, 0);
	
	//initialize the variables
	vx = 0;
	vy = 0;
	va = 0;
	displacement = 0;

	struct Sample {
		uint64_t sensor_ts;
		float encoder_1, encoder_2, encoder_pivot, velocity_1, velocity_2, velocity_pivot;
	};
	std::vector<Sample> samples(nWheels);
	std::vector<uint64_t> timestamps(nWheels);
	for (int i = 0; i < nWheels; i++) {
		volatile txpdo1_t* swData = driver->getWheelProcessData(i);
		Sample& sm = samples[i];
		int repcnt = 3;
		// read mutiple times if data has changed while reading
		do
		{
			sm.sensor_ts = swData->sensor_ts;
			sm.encoder_1 = swData->encoder_1;
			sm.encoder_2 = swData->encoder_2;
			sm.encoder_pivot = swData->encoder_pivot;
			sm.velocity_1 = swData->velocity_1;
			sm.velocity_2 = swData->velocity_2;
			sm.velocity_pivot = swData->velocity_pivot;
		} while ((sm.sensor_ts != swData->sensor_ts) && --repcnt);
		timestamps[i] = sm.sensor_ts;
	}
	OdometryFreshnessTracker::Update freshness = freshnessTracker->update(timestamps);

	double encoderDt = 0.0;
	for (int i = 0; i < nWheels; i++) {
		const float encoder_1 = samples[i].encoder_1, encoder_2 = samples[i].encoder_2;
		const float encoder_pivot = samples[i].encoder_pivot;
		const float velocity_1 = samples[i].velocity_1, velocity_2 = samples[i].velocity_2;
		const float velocity_pivot = samples[i].velocity_pivot;
		double delta_ts = freshnessTracker->deltaNs(i) * 0.000000001;

		double wl, wr, wp;
		// The encoder delta is only trusted over a plausible interval: never
		// across the gap after stale data (a drive re-init can restart
		// sensor_ts near zero), and not without a new sample (delta_ts == 0).
		if(!isEncoderDeltaUsable(delta_ts, freshness.resync, encoderDeltaLimit))
		{
			wl = velocity_1;
			wr = -velocity_2;
			wp = velocity_pivot;
			double fallbackDt = fallbackDtSec(delta_ts, freshness.resync, maxHeldGap, LOOP_PERIOD_SEC);
			if (fallbackDt > encoderDt) encoderDt = fallbackDt;
		}
		else
		{
			wl = norm(encoder_1 - prev_left_enc[i]) / delta_ts;
			wr = -norm(encoder_2 - prev_right_enc[i]) / delta_ts;
			wp = norm(encoder_pivot - prev_pivot_enc[i]) / delta_ts;
			if(fabs(velocity_pivot) > 10 * M_PI) wp = velocity_pivot;
			// Every wheel's baseline advances together, so the deltas agree to
			// within the sync jitter; the pose needs one dt for the fused velocity.
			if (delta_ts > encoderDt) encoderDt = delta_ts;
		}
		// Held steps keep the old baseline so the next used step's delta covers
		// the gap the pose skipped.
		if (!holdOdometry(freshness.stale, freshness.noNewSample)) {
			prev_left_enc[i] = encoder_1;
			prev_right_enc[i] = encoder_2;
			prev_pivot_enc[i] = encoder_pivot;
		}
		if (wheelConfigs[i].reverseVelocity) {
			wl *= -1.0;
			wr *= -1.0;
		}
		double theta = norm(encoder_pivot - wheelConfigs[i].a); // encoder_offset can be obtained from the yaml file or smartWheelDriver class
		double sin_theta = sin(theta);
		double cos_theta = cos(theta);
		displacement = (wl + wr) * delta_ts; // for liveliness check

		// calculate velocity components in wheel frame
		double cx = 0.5 * (0.5 * wheelConfigs[i].model.diameter) * (wl + wr);
		double cy = wp * wheelConfigs[i].model.casteroffset;

		// transform to robot frame at pivot position
		rx[i] = (cx * cos_theta) - (cy * sin_theta);
		ry[i] = (cx * sin_theta) + (cy * cos_theta);
		// sum cartesian velocities
		vx += rx[i];
		vy += ry[i];
	}
	if (encoderDt > 0.0) dt = encoderDt;

	// calcultate cartesian velocity of robot center from average of all wheel units
	if (nWheels > 1) {
		vx /= nWheels;
		vy /= nWheels;
	}

	double d_sum = 0.0;
	double v_sum = 0.0;
	for (int i = 0; i < nWheels; i++) {
		// substract cartesian robot velocity from wheel velocity.
		// use reminder to calculate robot angular velocity.
		rx[i] -= vx;
		ry[i] -= vy;

		// distance from wheel pivot position to robot center
		double d = sqrt((wheelConfigs[i].x * wheelConfigs[i].x) + (wheelConfigs[i].y * wheelConfigs[i].y));
		if(d > 0.0)
		{
			double cos_gamma = wheelConfigs[i].x / d;
			double sin_gamma = wheelConfigs[i].y / d;
			// Angular velocity around robot center equals the normal velocity component,
			// from wheel pivot position to robot center, divided by distance to robot center.
			// By not dividing directly by the distance, but doing this later for with sum of distances,
			// the sensitivity for wheel positions close to the robot center are compensated. 
			//
			//      v1   v2   v3   v4
			//      -- + -- + -- + --
			//      d1   d2   d3   d4                               v1 + v2 + v3 + v4
			// va = ------------------ (standard solution)     va = ----------------- (weighted solution)
			//          nWheels                                     d1 + d2 + d3 + d4   
			v_sum += (ry[i] * cos_gamma) - (rx[i] * sin_gamma);
			d_sum += d; 
		}
	}
	// calculate angular velocity of robot center from average of all wheel units
	if(d_sum > 0.0) va = v_sum / d_sum;

	if (holdOdometry(freshness.stale, freshness.noNewSample)) {
		vx = 0;
		vy = 0;
		va = 0;
		displacement = 0;
	}
	return freshness;
}

void PlatformDriverROS::calculateRobotPose2(double vx, double vy, double va, double dt) {
	double dx = vx * dt;
	double dy = vy * dt;
	// simplify circle movement by line between last and current location
	double a_average = norm(odoma + (0.5 * va * dt));
	double sin_a = sin(a_average);
	double cos_a = cos(a_average);
	// transform displacement to odom frame
	odomx += (dx * cos_a) - (dy * sin_a);
	odomy += (dx * sin_a) + (dy * cos_a);
	odoma = norm(odoma + (va * dt));
}

void PlatformDriverROS::publishOdometry(double vx, double vy, double va, bool wheelDataStale) {
	tf2::Quaternion odom_quat;
	odom_quat.setRPY(0, 0, odoma);
	nav_msgs::msg::Odometry odom;
	odom.header.stamp = nh->get_clock()->now();
	//odom.header.seq = sequence_id++;
	odom.header.frame_id = odomFrame;
	odom.child_frame_id = baseFrame;
	odom.pose.covariance[0] = 1e-3;
	odom.pose.covariance[7] = 1e-3;
	odom.pose.covariance[8] = 0.0;
	odom.pose.covariance[14] = 1e6;
	odom.pose.covariance[21] = 1e6;
	odom.pose.covariance[28] = 1e6;
	odom.pose.covariance[35] = 1e3;
	odom.twist.covariance[0] = twistCovariance(wheelDataStale, 1e-3, staleTwistCovariance);
	odom.twist.covariance[7] = twistCovariance(wheelDataStale, 1e-3, staleTwistCovariance);
	odom.twist.covariance[8] = 0.0;
	odom.twist.covariance[14] = 1e6;
	odom.twist.covariance[21] = 1e6;
	odom.twist.covariance[28] = 1e6;
	odom.twist.covariance[35] = twistCovariance(wheelDataStale, 1e3, staleTwistCovariance);
	odom.pose.pose.position.x = odomx;
	odom.pose.pose.position.y = odomy;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = tf2::toMsg(odom_quat);	
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = vy;
	odom.twist.twist.angular.z = va;
	odomPublisher->publish(odom);
}
		
void PlatformDriverROS::createOdomToBaseLinkTransform(geometry_msgs::msg::TransformStamped& odom_trans) {
	tf2::Quaternion odom_quat;
	odom_quat.setRPY(0, 0, odoma);
	odom_trans.header.stamp = nh->get_clock()->now();
	odom_trans.header.frame_id = odomFrame;
	odom_trans.child_frame_id = baseFrame;
	odom_trans.transform.translation.x = odomx;
	odom_trans.transform.translation.y = odomy;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = tf2::toMsg(odom_quat);
}

void PlatformDriverROS::publishProcessDataInput() {
	kelo_tulip::msg::KeloDrivesInput msg;
	for (int i = 0; i < nWheels; i++) {
		txpdo1_t* swData = driver->getWheelProcessData(i);
		kelo_tulip::msg::KeloDriveInput wheel;
		wheel.ddata = swData->ddata;
		wheel.status1 = swData->status1;
		wheel.status2 = swData->status2;
		wheel.sensor_ts = swData->sensor_ts;
		wheel.setpoint_ts = swData->setpoint_ts;
		wheel.encoder_1 = swData->encoder_1;
		wheel.velocity_1 = swData->velocity_1;
		wheel.current_1_d = swData->current_1_d;
		wheel.current_1_q = swData->current_1_q;
		wheel.current_1_u = swData->current_1_u;
		wheel.current_1_v = swData->current_1_v;
		wheel.current_1_w = swData->current_1_w;
		wheel.voltage_1 = swData->voltage_1;
		wheel.voltage_1_u = swData->voltage_1_u;
		wheel.voltage_1_v = swData->voltage_1_v;
		wheel.voltage_1_w = swData->voltage_1_w;
		wheel.temperature_1 = swData->temperature_1;
		wheel.encoder_2 = swData->encoder_2;
		wheel.velocity_2 = swData->velocity_2;
		wheel.current_2_d = swData->current_2_d;
		wheel.current_2_q = swData->current_2_q;
		wheel.current_2_u = swData->current_2_u;
		wheel.current_2_v = swData->current_2_v;
		wheel.current_2_w = swData->current_2_w;
		wheel.voltage_2 = swData->voltage_2;
		wheel.voltage_2_u = swData->voltage_2_u;
		wheel.voltage_2_v = swData->voltage_2_v;
		wheel.voltage_2_w = swData->voltage_2_w;
		wheel.temperature_2 = swData->temperature_2;
		wheel.encoder_pivot = swData->encoder_pivot;
		wheel.velocity_pivot = swData->velocity_pivot;
		wheel.voltage_bus = swData->voltage_bus;
		wheel.imu_ts = swData->imu_ts;
		wheel.accel_x = swData->accel_x;
		wheel.accel_y = swData->accel_y;
		wheel.accel_z = swData->accel_z;
		wheel.gyro_x = swData->gyro_x;
		wheel.gyro_y = swData->gyro_y;
		wheel.gyro_z = swData->gyro_z;
		wheel.temperature_imu = swData->temperature_imu;
		wheel.pressure = swData->pressure;
		wheel.current_in = swData->current_in;
		msg.wheels.push_back(wheel);
	}
	processDataInputPublisher->publish(msg);
}

void PlatformDriverROS::publishBattery() {
	std_msgs::msg::Float32 msg;
	double volt = 0;
	for (unsigned int i = 0; i < wheelConfigs.size(); i++) {
		double x = driver->getWheelProcessData(i)->voltage_bus;
		if (x > volt)
			volt = x;
	}	
	msg.data = volt;
	batteryPublisher->publish(msg);
}

void PlatformDriverROS::publishIMU() {
	for (unsigned int i=0; i<wheelConfigs.size(); i++) {
		// TODO : need to add wheel number to IMU data to prevent confusion
		// txpdo1_t* swData = driver->getWheelProcessData(i);
		// sensor_msgs::Imu imu;
		// imu.angular_velocity.x = swData->gyro_x;
		// imu.angular_velocity.y = swData->gyro_y;
		// imu.angular_velocity.z = swData->gyro_z;
		// imu.linear_acceleration.x = swData->accel_x;
		// imu.linear_acceleration.y = swData->accel_y;
		// imu.linear_acceleration.z = swData->accel_z;
		// imuPublisher.publish(imu);
	}
}

void PlatformDriverROS::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy) {
	joyCallbackImpl(joy);
}

// cmd_vel is the only velocity input this driver accepts; a velocity arbiter
// node is expected to be the sole publisher of that topic and to be
// responsible for arbitrating manual/joypad input against autonomous
// commands, obstacle and fault latches. This driver must never read the
// joypad to drive the wheels directly -- doing so let a held deadman button
// bypass the arbiter's clamp, ramp and latches entirely. The only legitimate
// use of /joy left here is the activeByJoypad startup gate below, which only
// ever flips a one-time READY-to-ACTIVE latch and never sets a velocity.
void PlatformDriverROS::joyCallbackImpl(const sensor_msgs::msg::Joy::SharedPtr joy) {
	if (!activeByJoypad)
		return;

	// Matches the deadman button convention used by the velocity arbiter
	// (button 5 / R1). Bounds-checked: a joypad reporting fewer buttons than
	// expected must not read out of range on this safety-critical host.
	const size_t deadmanButton = 5;
	if (deadmanButton < joy->buttons.size() && joy->buttons[deadmanButton])
		driver->setCanChangeActive();
}

void PlatformDriverROS::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
	cmdVelWatchdog.kick(CommandWatchdog::Clock::now());
	VelocityCommand command{msg->linear.x, msg->linear.y, msg->angular.z};
	if (!sanitizeCommand(command))
		RCLCPP_WARN_THROTTLE(nh->get_logger(), *nh->get_clock(), 1000,
			"Non-finite /cmd_vel (%f, %f, %f), commanding zero velocity", msg->linear.x, msg->linear.y,
			msg->angular.z);
	driver->setTargetVelocity(command.vx, command.vy, command.va);
}

void PlatformDriverROS::resetCallback(const std_msgs::msg::Empty::SharedPtr msg) const {
	// only error flags are resetted so far
	RCLCPP_INFO(nh->get_logger(), "Reset error flags.");
	driver->resetErrorFlags();
}

void PlatformDriverROS::enableCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) const {
	driver->setWheelsEnable(msg->data);
}


} //namespace kelo
