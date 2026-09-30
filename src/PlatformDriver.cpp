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

#include "kelo_tulip/PlatformDriver.h"
#include <algorithm>
#include <iostream>
#include "kelo_tulip/RtLog.h"

extern "C" {
#include "kelo_tulip/soem/ethercat.h"
#include "kelo_tulip/soem/ethercattype.h"
#include "nicdrv.h"
#include "kelo_tulip/soem/ethercatbase.h"
#include "kelo_tulip/soem/ethercatmain.h"
#include "kelo_tulip/soem/ethercatconfig.h"
#include "kelo_tulip/soem/ethercatcoe.h"
#include "kelo_tulip/soem/ethercatdc.h"
#include "kelo_tulip/soem/ethercatprint.h"
}

namespace kelo {

namespace {
// Per 1 ms cycle. The wheel setpoints of a holding platform ramp down this
// fast, 50 rad/s^2: a full-speed wheel (35 rad/s) is at rest in 0.7 s and a
// typical one in about 0.1 s, since a wheel is disabled and the platform must
// not keep driving on the rest. The ramp keeps a stop from being an instantaneous
// zero; the drive's current limit bounds the braking torque either way.
constexpr float kHoldSetpointStep = 0.05f;
}  // namespace

PlatformDriver::PlatformDriver(const std::vector<WheelConfig>& wheelConfigs, const std::vector<WheelData>& wheelData)
	: wheelConfigs(wheelConfigs)
	, wheelData(wheelData)
{
	if (wheelConfigs.size() != wheelData.size()) { // should not happen
		std::cout << "Error: wheel data structs have inconsistent size\n";
	}

	nWheels = wheelConfigs.size();
	wheelEnabled.resize(nWheels, true);
	operatorEnable.reset(new std::atomic<bool>[nWheels]);
	for (int i = 0; i < nWheels; i++)
		operatorEnable[i].store(true);
	recoveryAllowsEnable.resize(nWheels, true);

	// controller parameters
	maxvlin = 1.5;
	maxva = 1.0;
	maxvlinacc = 0.0025; // per msec, same value for dec
	maxangleacc = 0.01; // at vlin=0, per msec, same value for dec
	maxvaacc = 0.01; // per msec, same value for dec
	
	initTolerance = 30;
	initCounter = 0;
	
	// status variables
	state = DRIVER_STATE_INIT;
	statusError = false;
	timestampError = false;
	canChangeActive = false;
	showedMessageChangeActive = false;
	stepCount = 0;

	wheelsetpointmin = 0.01;
	wheelsetpointmax = 35.0;

	current_ts = 0;
	swErrorCount = 0;
	ethercatWkcError = false;

	flagReconnectSlave = false;

	encoderInitialized = false;
	encCalibrationTolerance = 20.0;
	sum_encoder.resize(nWheels, std::vector<double> (2, 0));
	abs_sum_encoder.resize(nWheels, std::vector<double> (2, 0));
	prev_encoder.resize(nWheels, std::vector<double> (2, 0));
	wheel_setpoint_ts.resize(nWheels, 0);
	wheel_sensor_ts.resize(nWheels, 0);
	processData.resize(nWheels);
	lastProcessData.resize(nWheels);

	velocityPlatformController.initialise(wheelConfigs);

	initAttemptStartStep = 0;
	initResets = 0;
	maxInitResets = 3;
	initTimeoutSteps = 500;

	clockStart = RecoveryClock::now();
	lastRealMs = nowMs();
	recoveryMachines.assign(nWheels, WheelRecoveryMachine(recoveryConfig, recoveryClockMs));
	linkMonitors.resize(nWheels);
	wheelHolding.resize(nWheels, false);
	setpointSlew.resize(2 * nWheels);
	wheelSetpoints.assign(2 * nWheels, 0.0f);
	releaseStep = kHoldSetpointStep;
}

PlatformDriver::~PlatformDriver() {
}

PlatformDriver::PlatformDriver(const PlatformDriver&) {
}

bool PlatformDriver::initEtherCAT2(ecx_contextt* ecx_context, int ecx_slavecount) {
	this->ecx_contextp = ecx_context;
	this->ecx_slavecount = ecx_slavecount;
	return true;
}

bool PlatformDriver::initEtherCAT(ec_slavet* ecx_slaves, int ecx_slavecount) {
	this->ecx_slaves = ecx_slaves;

	std::cout << "PlatformDriver InitEtherCAT\n";
	for (unsigned int i = 0; i < wheelConfigs.size(); i++) {
		int slave = wheelConfigs[i].ethercatNumber;
		std::cout << "Wheel #" << i << " is slave #" << slave << std::endl; 
		if (slave > ecx_slavecount) { // slaves start index 1
			std::cout << "Found only " << ecx_slavecount << " EtherCAT slaves, but config requires at least " << slave << std::endl; 
			return false;
		}

		if (ecx_slaves[slave].eep_id != 24137745 && ecx_slaves[slave].eep_id != 0 && ecx_slaves[slave].eep_id != 0x17010091 && 
			ecx_slaves[slave].eep_id != 0x02001001  && ecx_slaves[slave].eep_id != 0x10003205) {
			std::cout << "EtherCAT slave #" << i << " has wrong id: " << ecx_slaves[slave].eep_id << std::endl;
			return false;
		}
	}

	return true;
}

bool PlatformDriver::step() {
	const bool keepRunning = stepStateMachine();
	recordBlackBoxSample();
	return keepRunning;
}

bool PlatformDriver::stepStateMachine() {
	stepCount++;
	lastProcessData = processData;
	for (int i = 0; i < nWheels; i++)
		processData[i] = *getWheelProcessData(i);

	// TODO check if should take timestamp differently, or from each wheel separately
	if (nWheels > 0)
		current_ts = processData[0].sensor_ts; // TODO: atleast use firstWheel
	
	updateStatusError();
	updateEncoders();
	for (int i = 0; i < nWheels; i++)
		applyWheelEnable(i);

	switch (state) {
		case DRIVER_STATE_INIT:   return stepInit();
		case DRIVER_STATE_READY:  return stepReady();
		case DRIVER_STATE_ACTIVE: return stepActive();
		case DRIVER_STATE_ERROR:  return stepError();
		default: 
			doStop();
	}

	return true;
}

// in state init: wait and don't move until all wheels are enabled
bool PlatformDriver::stepInit() {
	doStop();

	bool ready = true;
	for (int wheel = 0; wheel < nWheels; wheel++)
		if (!hasWheelStatusEnabled(wheel) || hasWheelStatusError(wheel))
			ready = false;
	
	if (ready) {
		state = DRIVER_STATE_READY;
		RtLog::instance().push("PlatformDriver from INIT to READY");
		resetErrorFlags();
	}

	switch (initTimeoutAction(ready, static_cast<unsigned int>(stepCount - initAttemptStartStep),
		initTimeoutSteps, initResets, maxInitResets)) {
		case InitAction::ResetWheels:
			initResets++;
			RtLog::instance().pushf("Wheels not ready after %u steps, re-running the start sequence (%u/%u).",
				initTimeoutSteps, initResets, maxInitResets);
			// doStop sends disable again until initCounter passes initTolerance, then enable.
			initCounter = 0;
			initAttemptStartStep = stepCount;
			// As on a fresh start, so hasWheelStatusError's calibration allowance applies again.
			for (int wheel = 0; wheel < nWheels; wheel++)
				abs_sum_encoder[wheel].assign(2, 0.0);
			break;
		case InitAction::GiveUp:
			RtLog::instance().push("Stopping platform driver, because wheels don't become ready.");
			return false;
		case InitAction::Wait:
			break;
	}

	return true;
}

bool PlatformDriver::stepReady() {
	doStop();

	if (!statusError) {
		if (canChangeActive) {
			state = DRIVER_STATE_ACTIVE;
			// Recovery has not been watching until now: start its latch clock here.
			lastRealMs = nowMs();
			recoveryMachines.assign(nWheels, WheelRecoveryMachine(recoveryConfig, recoveryClockMs));
			RtLog::instance().push("PlatformDriver from READY to ACTIVE");
		}
		else if (!showedMessageChangeActive) {
			RtLog::instance().push("platform driver is ready, but waiting for signal to become active.");
			showedMessageChangeActive = true;
		}		
	}

	return true;
}

bool PlatformDriver::stepActive() {
	doControl();
	// A wheel recovery gave up on is disabled; driving on the others is not
	// safe, so stop EtherCAT and let the stack restart.
	if (anyWheelFailed()) {
		RtLog::instance().push("Stopping platform driver, because a wheel could not be recovered.");
		return false;
	}
	return true;
}

void PlatformDriver::applyWheelEnable(unsigned int wheel) {
	wheelEnabled[wheel] = effectiveWheelEnable(operatorEnable[wheel].load(), recoveryAllowsEnable[wheel]);
	velocityPlatformController.setWheelActive(wheel, wheelEnabled[wheel]);
}

bool PlatformDriver::anyWheelFailed() const {
	for (int i = 0; i < nWheels; i++)
		if (recoveryMachines[i].failed())
			return true;
	return false;
}

bool PlatformDriver::stepError() {
	// could check some condition if error has been resolved and change to ready state
	doStop();
	return true;
}

void PlatformDriver::setTargetVelocity(double vx, double vy, double va) {
	velocityPlatformController.setPlatformTargetVelocity(vx, vy, va);
}

void PlatformDriver::setCanChangeActive() {
	canChangeActive = true;
}

void PlatformDriver::setMaxvlin(double x) {
	maxvlin = x;
	velocityPlatformController.setPlatformMaxLinVelocity(x);
}

double PlatformDriver::getMaxvlin() {
	return maxvlin;
}

void PlatformDriver::setMaxva(double x) {
	maxva = x;
	velocityPlatformController.setPlatformMaxAngVelocity(x);
}

double PlatformDriver::getMaxva() {
	return maxva;
}

void PlatformDriver::setMaxvlinacc(double x) {
	maxvlinacc = x;
	velocityPlatformController.setPlatformMaxLinAcceleration(x);
}

void PlatformDriver::setMaxangleacc(double x) {
	maxangleacc = x;
}

void PlatformDriver::setMaxvaacc(double x) {
	maxvaacc = x;
	velocityPlatformController.setPlatformMaxAngAcceleration(x);
}

void PlatformDriver::setMaxvlindec(double x) {
	velocityPlatformController.setPlatformMaxLinDeceleration(x);
}

void PlatformDriver::setMaxvadec(double x) {
	velocityPlatformController.setPlatformMaxAngDeceleration(x);
}

void PlatformDriver::setCurrentShaping(const CurrentShapingConfig& config) {
	velocityPlatformController.setCurrentShaping(config);
	releaseStep = releaseStepPerCycle(config.slewRateRadPerSecSq, kHoldSetpointStep);
}

void PlatformDriver::reconnectSlave(int slave) {
	if(slave >= 0) flagReconnectSlave = true;
}

txpdo1_t* PlatformDriver::getWheelProcessData(unsigned int wheel) {
	// TODO: thread synchronization
	int slave = wheelConfigs[wheel].ethercatNumber;
	return (txpdo1_t*) ecx_slaves[slave].inputs;
}

void PlatformDriver::setWheelProcessData(unsigned int wheel, rxpdo1_t* data) {
	// TODO: thread synchronization
	int slave = wheelConfigs[wheel].ethercatNumber;
	rxpdo1_t* ecData = (rxpdo1_t*) ecx_slaves[slave].outputs;
	*ecData = *data;
}

std::vector<double> PlatformDriver::getEncoderValue(int idx) {
	if (idx < 0 || idx >= nWheels) {
		std::cout << "Failed to return encoder value. Encoder index does not match" << std::endl;
		return std::vector<double>();
	}
	
	return sum_encoder[idx];
}

const	WheelData* PlatformDriver::getWheelData(unsigned int wheel) {
	return &wheelData[wheel];
}

bool PlatformDriver::hasWheelStatusEnabled(unsigned int wheel) {
	int status1 = processData[wheel].status1;
	return (status1 & STAT1_ENABLED1) > 0 && (status1 & STAT1_ENABLED2) > 0;
}

bool PlatformDriver::hasWheelStatusError(unsigned int wheel) {
	const int STATUS1a = 3;
	const int STATUS1b = 63;
	const int STATUS1disabled = 60;
	const int STATUS2 = 2051;

	int status1 = processData[wheel].status1;
	int status2 = processData[wheel].status2;

	//return (status1 != STATUS1a && status1 != STATUS1b && status1 != STATUS1disabled) || (status2 != STATUS2);
	return (status1 != STATUS1a && status1 != STATUS1b && status1 != STATUS1disabled) ||
	       (status2 != STATUS2 && abs_sum_encoder[wheel][0] > encCalibrationTolerance && abs_sum_encoder[wheel][1] > encCalibrationTolerance);
}

void PlatformDriver::updateStatusError() {
	for (int i = 0; i < nWheels; i++) {
		if (hasWheelStatusError(i)) {
			int s1 = processData[i].status1;
			int s2 = processData[i].status2;		
			if (!statusError) {
				RtLog::instance().pushf("Status error: wheel=%d, status1=%d, status2=%d", i, s1, s2);
				statusError = true;
			}
		}

		if (!hasWheelStatusEnabled(i) && state != DRIVER_STATE_INIT) {
			int s1 = processData[i].status1;
			int s2 = processData[i].status2;		
			if (!statusError) {
				RtLog::instance().pushf("Wheel got disabled: wheel=%d, status1=%d, status2=%d", i, s1, s2);
				statusError = true;
			}
		}
	}
}

int PlatformDriver::getDriverStatus() {
	int status = (int) state;

	int errorTimestampCrit = checkSmartwheelTimestamp();
	if (errorTimestampCrit == 3)
		status = DRIVER_STATE_ERROR;

	if (timestampError)
		status |= DRIVER_ERROR_TIMESTAMP;

	if (ethercatWkcError)
		status |= DRIVER_ERROR_ETHERCAT_WKC;

	if (statusError)
		status |= DRIVER_ERROR_STATUS;

	return status;
}

void PlatformDriver::resetErrorFlags() {
	ethercatWkcError = false;
	statusError = false;
	timestampError = false;

	for (int i = 0; i < nWheels; i++) {
		txpdo1_t* swData = getWheelProcessData(i);
		wheel_sensor_ts[i] = swData->sensor_ts;
		wheel_setpoint_ts[i] = swData->setpoint_ts;
	}
}

void PlatformDriver::setWheelsEnable(std::vector<int> values) {
	if (static_cast<size_t>(nWheels) != values.size()) {
		std::cout << "Number of wheels do not match. Ignoring enable command" << std::endl;
		return;
	}

	for (int i = 0; i < nWheels; i++) {
		if (values[i] == 1)
			operatorEnable[i].store(true);
		else if (values[i] == 0)
			operatorEnable[i].store(false);
		else
			std::cout << "wheel enable value is invalid. Ignoring enable value" << std::endl;
	}
}

int PlatformDriver::checkSmartwheelTimestamp() {
	bool swOK = true;
	for (int i = 0; i < nWheels; i++) {
		txpdo1_t* swData = getWheelProcessData(i);
		bool error = false;

		if (wheel_sensor_ts[i] < swData->sensor_ts)
			wheel_sensor_ts[i] = swData->sensor_ts;
		else
			error = true;

		if (wheel_setpoint_ts[i] < swData->setpoint_ts)
			wheel_setpoint_ts[i] = swData->setpoint_ts;
		else
			error = true;

		if (error != wheelData[i].errorTimestamp) {
			if (error)
				std::cout << "Timestamp error wheel " << i << ".\n";
			else
				std::cout << "Timestamp wheel " << i << " is ok again.\n";
			wheelData[i].errorTimestamp = error;
		}

		swOK = swOK & !error;
	}
	
	int result = 4;
	if (swOK) {
		result = 2;
		timestampError = false;
	}

	if (!swOK)
		timestampError = true;

	return result;
}

void PlatformDriver::SetState(int wheel, uint16_t state) {
	int slave = wheelConfigs[wheel].ethercatNumber;
	std::cout << "SetState wheel " << wheel << " slave " << slave << " state " << std::hex << state << std::endl;
	ecx_context.slavelist[slave].state = state;
	ecx_writestate(ecx_contextp, slave);
	std::cout << " SetState done." << std::endl;
}

void PlatformDriver::updateEncoders() {
	if(!encoderInitialized) {
		for (int i = 0; i < nWheels; i++) {
			txpdo1_t* wData = getWheelProcessData(i);
			prev_encoder[i][0] = wData->encoder_1;
			prev_encoder[i][1] = wData->encoder_2;
		}
		encoderInitialized = true;
	}
	
	//count accumulative encoder value
	for (int i = 0; i < nWheels; i++) {
		txpdo1_t* wData = getWheelProcessData(i);
		double curr_encoder1 = wData->encoder_1;
		double curr_encoder2 = wData->encoder_2;
		if (fabs(curr_encoder1 - prev_encoder[i][0]) > M_PI) {
			if (curr_encoder1 < prev_encoder[i][0]) {
				sum_encoder[i][0] += curr_encoder1 - prev_encoder[i][0] + 2 * M_PI;
				abs_sum_encoder[i][0] += fabs(curr_encoder1 - prev_encoder[i][0] + 2 * M_PI);
			} else {
				sum_encoder[i][0] += curr_encoder1 - prev_encoder[i][0] - 2 * M_PI;
				abs_sum_encoder[i][0] += fabs(curr_encoder1 - prev_encoder[i][0] - 2 * M_PI);
			}
		} else {
			sum_encoder[i][0] += curr_encoder1 - prev_encoder[i][0];
			abs_sum_encoder[i][0] += fabs(curr_encoder1 - prev_encoder[i][0]);
		}
			
		if (fabs(curr_encoder2 - prev_encoder[i][1]) > M_PI) {
			if (curr_encoder2 < prev_encoder[i][1]) {
				sum_encoder[i][1] += curr_encoder2 - prev_encoder[i][1] + 2 * M_PI;
				abs_sum_encoder[i][1] += fabs(curr_encoder2 - prev_encoder[i][1] + 2 * M_PI);
			} else {
				sum_encoder[i][1] += curr_encoder2 - prev_encoder[i][1] - 2 * M_PI;
				abs_sum_encoder[i][1] += fabs(curr_encoder2 - prev_encoder[i][1] - 2 * M_PI);
			}
		} else {
			sum_encoder[i][1] += curr_encoder2 - prev_encoder[i][1];
			abs_sum_encoder[i][1] += fabs(curr_encoder2 - prev_encoder[i][1]);
		}
		prev_encoder[i][0] = curr_encoder1;
		prev_encoder[i][1] = curr_encoder2;
	}
}

void PlatformDriver::doStop() {
	rxpdo1_t rxdata;
	rxdata.timestamp = current_ts + 100 * 1000; // TODO
	rxdata.setpoint1 = 0;
	rxdata.setpoint2 = 0;

	for (int i = 0; i < nWheels; i++) {
		if (wheelEnabled[i] && initCounter > initTolerance)
			rxdata.command1 = COM1_ENABLE1 | COM1_ENABLE2 | COM1_MODE_VELOCITY;
		else {
			rxdata.command1 = COM1_MODE_VELOCITY;
		    initCounter++;
			resetErrorFlags();
		}
		rxdata.command2 = COM2_MODE_VELOCITY;
		rxdata.limit1_p = wheelConfigs[i].model.standbycurrent;
		rxdata.limit1_n = -wheelConfigs[i].model.standbycurrent;
		rxdata.limit2_p = wheelConfigs[i].model.standbycurrent;
		rxdata.limit2_n = -wheelConfigs[i].model.standbycurrent;

		setWheelProcessData(i, &rxdata);
	}
	setpointSlew.reset();
}

void PlatformDriver::doControl() {
	/* initialise struct to be sent to wheels */
	rxpdo1_t rxdata;
	rxdata.timestamp = current_ts + 100 * 1000; // TODO
	rxdata.setpoint1 = 0;
	rxdata.setpoint2 = 0;

	// update desired velocity of platform, based on target velocity and velocity ramps
	velocityPlatformController.calculatePlatformRampedVelocities();

	advanceRecoveryClock();
	for (int i = 0; i < nWheels; i++)
		doWheelRecovery(i);
	// A wheel that is recovering or whose slave is gone makes the platform
	// stop: the rest keep no purpose driving alone. Odometry reads zero
	// meanwhile for a lost slave (its inputs are zeroed, so its timestamp is
	// stale); that is accepted. On give-up stepActive ends the loop before the
	// ramp completes, which the drive's current limit and the restart cover.
	const bool hold = anyWheelHolding();

	for (int i = 0; i < nWheels; i++) {
		txpdo1_t* wheel_data = getWheelProcessData(i);
		float setpoint1, setpoint2;

		/* calculate wheel target velocity */
		velocityPlatformController.calculateWheelTargetVelocity(i, wheel_data->encoder_pivot,
                                                                setpoint2, setpoint1);
		setpoint2 *= -1; // because of inverted frame

		/* avoid sending close to zero values */
		if ( fabs(setpoint1) < wheelsetpointmin )
		{
			setpoint1 = 0;
		}
		if ( fabs(setpoint2) < wheelsetpointmin )
		{
			setpoint2 = 0;
		}

		/* avoid sending very large values */
		wheelSetpoints[2 * i] = Utils::clip(setpoint1, wheelsetpointmax, -wheelsetpointmax);
		wheelSetpoints[2 * i + 1] = Utils::clip(setpoint2, wheelsetpointmax, -wheelsetpointmax);
	}

	/* ramp all wheels to zero together while any wheel is recovering, never cut */
	setpointSlew.step(wheelSetpoints.data(), hold, kHoldSetpointStep, releaseStep, wheelSetpoints.data());

	for (int i = 0; i < nWheels; i++) {
		if (wheelEnabled[i])
			rxdata.command1 = COM1_ENABLE1 | COM1_ENABLE2 | COM1_MODE_VELOCITY;
		else
			rxdata.command1 = COM1_MODE_VELOCITY;

		rxdata.command2 = COM2_MODE_VELOCITY;
		rxdata.limit1_p = wheelConfigs[i].model.currentlimit;
		rxdata.limit1_n = -wheelConfigs[i].model.currentlimit;
		rxdata.limit2_p = wheelConfigs[i].model.currentlimit;
		rxdata.limit2_n = -wheelConfigs[i].model.currentlimit;

		/* send calculated target velocity values to EtherCAT */
		rxdata.setpoint1 = wheelSetpoints[2 * i];
		rxdata.setpoint2 = wheelSetpoints[2 * i + 1];

		setWheelProcessData(i, &rxdata);
	}
}

void PlatformDriver::advanceRecoveryClock() {
	constexpr double kMaxStepMs = 5.0;
	const double real = nowMs();
	recoveryClockMs += std::min(std::max(real - lastRealMs, 0.0), kMaxStepMs);
	lastRealMs = real;
}

double PlatformDriver::nowMs() const {
	return std::chrono::duration<double, std::milli>(RecoveryClock::now() - clockStart).count();
}

bool PlatformDriver::wheelLinkUp(unsigned int wheel) {
	if (!ecx_slaves)
		return true;
	const ec_slavet& slave = ecx_slaves[wheelConfigs[wheel].ethercatNumber];
	return linkMonitors[wheel].update(recoveryClockMs, slave.islost != FALSE, slave.state == EC_STATE_OPERATIONAL);
}

bool PlatformDriver::anyWheelHolding() const {
	for (int i = 0; i < nWheels; i++)
		if (wheelHolding[i])
			return true;
	return false;
}

void PlatformDriver::doWheelRecovery(unsigned int wheel) {
	const bool operatorWants = operatorEnable[wheel].load();
	const txpdo1_t& data = processData[wheel];
	const WheelRecoveryMachine::Input input{recoveryClockMs, operatorWants,
		wheelNeedsRecovery(data.status1, data.status2, operatorWants), wheelLinkUp(wheel),
		wheelStatusSane(data.status1)};

	const WheelRecoveryMachine::Output out = recoveryMachines[wheel].step(input);
	recoveryAllowsEnable[wheel] = out.allowsEnable;
	wheelHolding[wheel] = out.holdAtZero;
	logRecoveryEvents(wheel, out.events);
	applyWheelEnable(wheel);
}

// Runs on the EtherCAT thread: lines go to the RtLog, never to stdout.
void PlatformDriver::logRecoveryEvents(unsigned int wheel, unsigned int events) {
	if (events == 0)
		return;
	const auto has = [events](RecoveryEvent event) { return (events & event) != 0; };
	const txpdo1_t& data = processData[wheel];
	const int slave = wheelConfigs[wheel].ethercatNumber;
	RtLog& log = RtLog::instance();
	const unsigned int w = wheel;

	if (has(RECOVERY_EVENT_ABORTED_BY_OPERATOR))
		log.pushf("Wheel %u recovery aborted: wheel disabled by operator", w);
	if (has(RECOVERY_EVENT_LINK_LOST))
		log.pushf("Wheel %u (slave %d) is not answering: holding all wheels, waiting up to %.0f ms for it", w, slave,
			recoveryConfig.linkLossWindowMs);
	if (has(RECOVERY_EVENT_LINK_RESTORED))
		log.pushf("Wheel %u (slave %d) answers again: status1=%u, status2=%u", w, slave, data.status1, data.status2);
	if (has(RECOVERY_EVENT_STARTED))
		log.pushf("Start wheel %u recovery", w);
	if (has(RECOVERY_EVENT_SUCCEEDED))
		log.pushf("Wheel %u recovery successful", w);
	if (has(RECOVERY_EVENT_ATTEMPT_FAILED))
		log.pushf("Wheel %u recovery failed", w);
	if (has(RECOVERY_EVENT_ATTEMPTS_RESET))
		log.pushf("Wheel %u recovery attempts are reset to 0.", w);
	if (has(RECOVERY_EVENT_LINK_LOSS_TIMEOUT))
		log.pushf("Wheel %u (slave %d) did not answer within %.0f ms", w, slave, recoveryConfig.linkLossWindowMs);
	if (has(RECOVERY_EVENT_GAVE_UP))
		log.pushf("Wheel %u could not be recovered. Stopping operation (status1=%u, status2=%u)", w, data.status1,
			data.status2);

	if (blackBox) {
		if (has(RECOVERY_EVENT_GAVE_UP))
			blackBox->trigger(DumpReason::WheelFailed);
		else if (has(RECOVERY_EVENT_STARTED) || has(RECOVERY_EVENT_LINK_LOST))
			blackBox->trigger(DumpReason::WheelRecovery);
	}
}

void PlatformDriver::recordBlackBoxSample() {
	if (!blackBox || !ecx_slaves)
		return;
	CycleSample sample{};
	sample.cycle = static_cast<std::uint64_t>(stepCount);
	sample.monotonicNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count();
	sample.wkc = blackBox->wkc();
	sample.expectedWkc = blackBox->expectedWkc();
	sample.driverState = static_cast<std::uint8_t>(state);
	const int recorded = std::min<int>(nWheels, kMaxBlackBoxWheels);
	sample.wheelCount = static_cast<std::uint8_t>(recorded);
	for (int i = 0; i < recorded; i++) {
		const int slave = wheelConfigs[i].ethercatNumber;
		const txpdo1_t& in = *reinterpret_cast<const txpdo1_t*>(ecx_slaves[slave].inputs);
		const rxpdo1_t& out = *reinterpret_cast<const rxpdo1_t*>(ecx_slaves[slave].outputs);
		WheelSample& w = sample.wheels[i];
		w.sensorTs = in.sensor_ts;
		w.voltageBus = in.voltage_bus;
		w.currentIn = in.current_in;
		w.current1q = in.current_1_q;
		w.current2q = in.current_2_q;
		w.status1 = in.status1;
		w.status2 = in.status2;
		w.command1 = out.command1;
		w.setpoint1 = out.setpoint1;
		w.setpoint2 = out.setpoint2;
		w.limit1p = out.limit1_p;
		w.limit2p = out.limit2_p;
		w.wheelState = static_cast<std::uint8_t>(recoveryMachines[i].state());
		w.flags = (ecx_slaves[slave].islost ? 0 : kWheelFlagLinkUp) | (wheelEnabled[i] ? kWheelFlagEnabled : 0);
	}
	blackBox->record(sample);
}


} //namespace kelo
