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

#include "kelo_tulip/DriverLiveness.h"
#include "kelo_tulip/EscDiagnosticsRos.h"
#include "kelo_tulip/EtherCATMaster.h"
#include "kelo_tulip/ParameterValidation.h"
#include "kelo_tulip/PlatformDriverROS.h"
#include "kelo_tulip/modules/RobileMasterBatteryROS.h"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <thread>

namespace {

// Set from the signal handler. rclcpp's own handler is not installed: it
// would shut the context down at once, and the exit must instead keep the
// EtherCAT loop alive until the wheels have been ramped to zero.
std::atomic<bool> exitRequested{false};
static_assert(std::atomic<bool>::is_always_lock_free, "signal handler needs a lock-free flag");

extern "C" void onExitSignal(int) {
	exitRequested.store(true);
}

void installExitHandlers() {
	struct sigaction action{};
	action.sa_handler = onExitSignal;
	// Slow calls during start-up (SOEM's socket I/O) resume instead of failing.
	action.sa_flags = SA_RESTART;
	sigemptyset(&action.sa_mask);
	sigaction(SIGINT, &action, nullptr);
	sigaction(SIGTERM, &action, nullptr);
}

// Margin past the driver's own deadline before the bus is released anyway:
// covers the settle and disable phases and an EtherCAT loop that stalls.
constexpr double kStopBackstopMarginSec = 1.0;

// Asks the modules to bring the drives to rest, then waits while the
// EtherCAT loop does it. Telemetry keeps flowing meanwhile; commands do not
// (nothing is spun, and the driver ignores them once stopping).
void stopDrives(rclcpp::Node::SharedPtr nh, kelo::EtherCATMaster* master,
	const std::vector<kelo::EtherCATModuleROS*>& rosModules) {
	if (master->hasStopped())
		return;
	const double timeoutSec = nh->has_parameter("stop_timeout") ?
		nh->get_parameter("stop_timeout").as_double() : kelo::DriverLimits().stopTimeoutSec;
	RCLCPP_WARN(nh->get_logger(), "Exiting: ramping the wheels to zero before releasing the drives (at most %.1f s)",
		timeoutSec);
	master->requestStop();

	const auto deadline = std::chrono::steady_clock::now() +
		std::chrono::duration_cast<std::chrono::steady_clock::duration>(
			std::chrono::duration<double>(timeoutSec + kStopBackstopMarginSec));
	rclcpp::WallRate rate(20.0);
	while (!master->hasStopped() && std::chrono::steady_clock::now() < deadline) {
		for (size_t i = 0; i < rosModules.size(); i++)
			rosModules[i]->step();
		rate.sleep();
	}
	if (master->hasStopped())
		RCLCPP_INFO(nh->get_logger(), "Drives stopped and disabled");
	else
		RCLCPP_ERROR(nh->get_logger(), "The EtherCAT loop did not finish the stop in time; releasing the bus");
}

}  // namespace

// create and configure one module
kelo::EtherCATModuleROS* createModule(rclcpp::Node::SharedPtr nh, std::string moduleType, std::string moduleName, std::string configTag) {
	kelo::EtherCATModuleROS* module = NULL;	
	if (moduleType == "robile_master_battery") {
		module = new kelo::RobileMasterBatteryROS();
	} else if (moduleType == "platform_driver") {
		module = new kelo::PlatformDriverROS();
	} else {
		std::cout << "Unknown module type: " << moduleType << std::endl;
		return NULL;
	}

	if (!module) {
		std::cout << "Module " << moduleName << " could not be created." << std::endl;
		return NULL;
	}

	// Initialize module and if not possible, delete it.
	if (!module->init(nh, configTag)) {
		std::cout << "Failed to initialize module " << moduleName << "." << std::endl;
		delete module;
		return NULL;
	}
	
	return module;
}

// Where the black box dumps go when the launch names no directory. With no home
// there is none: a shared, guessable directory is worse than no dumps.
std::string defaultDumpDir() {
	const char* rosHome = std::getenv("ROS_HOME");
	const char* home = std::getenv("HOME");
	if (rosHome && *rosHome)
		return std::string(rosHome) + "/kelo_tulip/ethercat_dumps";
	if (home && *home)
		return std::string(home) + "/.ros/kelo_tulip/ethercat_dumps";
	return "";
}

kelo::BlackBoxConfig readBlackBoxConfig(rclcpp::Node::SharedPtr nh) {
	kelo::BlackBoxConfig config;
	config.dir = nh->get_parameter("blackbox.dir").as_string();
	if (config.dir.empty())
		config.dir = defaultDumpDir();
	config.historyS = nh->get_parameter("blackbox.history_s").as_double();
	config.postTriggerS = nh->get_parameter("blackbox.post_trigger_s").as_double();
	config.minIntervalS = nh->get_parameter("blackbox.min_interval_s").as_double();
	config.maxFiles = static_cast<std::size_t>(std::max<int64_t>(0, nh->get_parameter("blackbox.max_files").as_int()));
	config.maxTotalBytes =
		static_cast<std::uint64_t>(std::max<int64_t>(0, nh->get_parameter("blackbox.max_total_mb").as_int())) *
		1024ull * 1024ull;
	return kelo::sanitizeBlackBoxConfig(config);
}

/*
// step through all modules
void stepModules(const rclcpp::TimerEvent&) {
		for (size_t i = 0; i < rosModules.size(); i++)
			rosModules[i]->step();
}
*/

int main (int argc, char** argv)
{
	rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
	installExitHandlers();
	auto nh = rclcpp::Node::make_shared("platform_driver");

	nh->declare_parameter("modules.list", std::vector<std::string>{}); 
	nh->declare_parameter("start_retry_delay", 0);
	nh->declare_parameter("robile_master_battery_ethercat_number", 0);
	nh->declare_parameter("device", "");
	nh->declare_parameter("enable_ethercat_recovery", false);
	// Last seconds of the 1 kHz loop, dumped to CSV on a communication error.
	nh->declare_parameter("blackbox.enabled", true);
	nh->declare_parameter("blackbox.dir", "");
	nh->declare_parameter("blackbox.history_s", 2.0);
	nh->declare_parameter("blackbox.post_trigger_s", 0.5);
	nh->declare_parameter("blackbox.min_interval_s", 10.0);
	nh->declare_parameter("blackbox.max_files", 100);
	nh->declare_parameter("blackbox.max_total_mb", 256);
	// ESC error counters per slave; 0 turns the read off.
	nh->declare_parameter("ethercat_diagnostics_period_s", 5.0);

	std::vector<kelo::EtherCATModuleROS*> rosModules;

	// create modules by iterating through struct in config
	std::string configModulesTag = "modules";
	std::vector<std::string> moduleList = nh->get_parameter(configModulesTag + ".list").as_string_array();
	for (unsigned int i = 0; i < moduleList.size(); i++) {
		nh->declare_parameter(configModulesTag + "." + moduleList[i] + ".type", "");
		std::string moduleName = moduleList[i];
		std::string moduleType = nh->get_parameter(configModulesTag + "." + moduleList[i] + ".type").as_string();
		std::string configTag = configModulesTag + "." + moduleName + ".";
		kelo::EtherCATModuleROS* module = createModule(nh, moduleType, moduleName, configTag);
		
		if (!module)
			return -1;
			
		rosModules.push_back(module);
	}

	// legacy config mode for master battery
	int robileMasterBatteryEthercatNumber = nh->get_parameter("robile_master_battery_ethercat_number").as_int();
	if (robileMasterBatteryEthercatNumber > 0) {	
		kelo::EtherCATModuleROS* module = new kelo::RobileMasterBatteryROS();
		if (!module || !module->init(nh, ""))
			return -1;
			
		rosModules.push_back(module);		
	}

	// collect EtherCAT modules
	std::vector<kelo::EtherCATModule*> etherCATmodules;
	for (size_t i = 0; i < rosModules.size(); i++)
		etherCATmodules.push_back(rosModules[i]->getEtherCATModule());

	// create and configure EtherCAT master
	std::string device = nh->get_parameter("device").as_string();
	int delayRetry = nh->get_parameter("start_retry_delay").as_int();
	bool enableEthercatRecovery = nh->get_parameter("enable_ethercat_recovery").as_bool();

	kelo::EtherCATMaster* master = new kelo::EtherCATMaster(device, etherCATmodules);
	if (!master) {
		std::cout << "Failed to create EtherCAT master." << std::endl;
		return -1;		
	}

	if (nh->get_parameter("blackbox.enabled").as_bool()) {
		const kelo::BlackBoxConfig blackBoxConfig = readBlackBoxConfig(nh);
		if (blackBoxConfig.dir.empty()) {
			RCLCPP_ERROR(nh->get_logger(), "No blackbox.dir, ROS_HOME or HOME: EtherCAT black box is off");
		} else {
			master->enableBlackBox(blackBoxConfig);
			RCLCPP_INFO(nh->get_logger(), "EtherCAT black box dumps to %s", blackBoxConfig.dir.c_str());
		}
	}
	master->enableEscDiagnostics(nh->get_parameter("ethercat_diagnostics_period_s").as_double());
	kelo::EscDiagnosticsPublisher escPublisher(nh, [master] { return master->escSnapshot(); });

	// initialize EtherCAT
	while (!master->initEthercat()) {
		if (delayRetry == 0 || exitRequested.load()) {
			RCLCPP_ERROR(nh->get_logger(), "Failed to initialize EtherCAT");
			return -1;
		}
		RCLCPP_ERROR(nh->get_logger(), "Failed to initialize EtherCAT, will retry in %d s.", delayRetry);
		std::this_thread::sleep_for(std::chrono::seconds(delayRetry));
	}
	
	// ROS main loop. WallRate: a wall-clock step must not stall it.
	rclcpp::WallRate rate(20.0); // hz
	int exitStatus = 0;
	while (rclcpp::ok() && !exitRequested.load()) {
		const bool reinitAttempted = enableEthercatRecovery && master->needsReinit();
		const bool reinitSucceeded = reinitAttempted && master->reinitializeEthercat();
		const kelo::LoopVerdict verdict =
			kelo::checkEthercatLiveness(reinitAttempted, reinitSucceeded, master->hasStopped());
		if (verdict != kelo::LoopVerdict::Continue) {
			RCLCPP_FATAL(nh->get_logger(), "%s; exiting so the stack is restarted.",
				verdict == kelo::LoopVerdict::ReinitFailed ?
				"EtherCAT reinitialization failed" : "EtherCAT communication has stopped");
			exitStatus = kelo::exitCode(verdict);
			break;
		}

		rclcpp::spin_some(nh);		
		escPublisher.publishIfNew();
		
		for (size_t i = 0; i < rosModules.size(); i++)
			rosModules[i]->step();
		
		rate.sleep();
	}

	// An orderly exit: the drives have no brakes, so they are brought to rest
	// before the master releases them. After an EtherCAT failure there is no
	// loop left to do it.
	if (exitStatus == 0)
		stopDrives(nh, master, rosModules);

	// The master's loop steps the modules, so it stops (and flushes the black
	// box) before they go.
	delete master;

	for (size_t i = 0; i < rosModules.size(); i++)
		delete rosModules[i];
	
	rclcpp::shutdown();
	return exitStatus;
}

