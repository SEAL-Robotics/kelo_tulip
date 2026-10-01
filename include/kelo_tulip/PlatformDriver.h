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


#ifndef PLATFORMDRIVER_H_
#define PLATFORMDRIVER_H_

extern "C" {
#include "kelo_tulip/soem/ethercattype.h"
#include "kelo_tulip/EtherCATModule.h"
#include "kelo_tulip/KeloDriveAPI.h"

#include "nicdrv.h"
#include "kelo_tulip/soem/ethercatbase.h"
#include "kelo_tulip/soem/ethercatmain.h"
#include "kelo_tulip/soem/ethercatconfig.h"
#include "kelo_tulip/soem/ethercatcoe.h"
#include "kelo_tulip/soem/ethercatdc.h"
#include "kelo_tulip/soem/ethercatprint.h"
}
#include "kelo_tulip/VelocityPlatformController.h"
#include "kelo_tulip/Utils.h"
#include "kelo_tulip/WheelConfig.h"
#include "kelo_tulip/WheelRecovery.h"
#include "kelo_tulip/EthercatBlackBox.h"
#include "kelo_tulip/StopSequence.h"
#include "kelo_tulip/VelocityCommand.h"
#include <boost/thread.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <fstream>

namespace kelo {

struct WheelData {
	bool enable;
	
	bool error;
	bool errorTimestamp;

};

enum DriverState {
	DRIVER_STATE_UNDEFINED = 0x00,
	DRIVER_STATE_ACTIVE = 0x01,
	DRIVER_STATE_READY = 0x02,
	DRIVER_STATE_INIT = 0x04,
	DRIVER_STATE_ERROR = 0x10,
};

enum DriverError {
	DRIVER_ERROR_UNSPECIFIED = 0x0100,
	DRIVER_ERROR_ETHERCAT_WKC = 0x0200,
	DRIVER_ERROR_TIMESTAMP = 0x0400,
	DRIVER_ERROR_STATUS = 0x0800,
	DRIVER_ERROR_STALL = 0x1000,
	DRIVER_ERROR_SLIP = 0x2000
};

class PlatformDriver : public EtherCATModule {
public:
	PlatformDriver(const std::vector<WheelConfig>& wheelConfigs, const std::vector<WheelData>& wheelData);
	virtual ~PlatformDriver();

	virtual bool initEtherCAT2(ecx_contextt* ecx_context, int ecx_slavecount); 

	virtual bool initEtherCAT(ec_slavet* ecx_slaves, int ecx_slavecount);
	virtual bool step();
	virtual void setBlackBox(EthercatBlackBox* box) { blackBox = box; }
	
	virtual bool stepInit();
	virtual bool stepReady();
	virtual bool stepActive();
	virtual bool stepError();
	virtual bool stepStopping();

	//! Thread-safe. A non-finite axis is a stop. Ignored once a stop has been
	//! requested. The EtherCAT cycle drops the target to zero once it is
	//! older than the command timeout, whatever the ROS side does.
	virtual void setTargetVelocity(double vx, double vy, double va);
	//! Seconds, in (0, CommandWatchdog::MAX_TIMEOUT_SEC]; other values are ignored.
	void setCommandTimeout(double sec);
	//! Shutdown ramp: decelerations (never above vlin_dec_max/va_dec_max)
	//! and the deadline for ramping and settling.
	void setStopParameters(double vlinDec, double vaDec, double timeoutSec);
	//! Thread-safe. Ramp to zero, settle, disable the drives, then end the
	//! EtherCAT loop (step() returns false). Cannot be undone.
	void requestStop() override;
	bool stopRequested() const { return stopRequestedFlag.load(); }

	txpdo1_t* getWheelProcessData(unsigned int wheel);
	void setWheelProcessData(unsigned int wheel, rxpdo1_t* data);

	void reconnectSlave(int slave);

	void setCanChangeActive();

	void setMaxvlin(double x);
	double getMaxvlin();
	void setMaxva(double x);
	double getMaxva();
	void setMaxvlinacc(double x);
	void setMaxangleacc(double x);
	void setMaxvaacc(double x);
	void setMaxvlindec(double x);
	void setMaxvadec(double x);
	void setCurrentShaping(const CurrentShapingConfig& config);
	void setFactorAngleaccVlin(double x);
	void setFractionVelTolerance(double x);
	void setFractionFactor(double x);

	void setVheadingControlp(double x);
	void setMaxvheading(double x);
	void setWheelsetpointacc(double x);
	void setWheelsetpointmin(double x);

	const	WheelData* getWheelData(unsigned int wheel);
	
	int getDriverStatus();

	void resetErrorFlags();
	void setWheelsEnable(std::vector<int> values);

	std::vector<double> getEncoderValue(int idx);

	void SetState(int wheel, uint16_t state);
		
protected:
	int checkSmartwheelTimestamp();
	void updateEncoders();
	void updateSetpoints();
	virtual void doStop();
	virtual void doControl();
	//! Zero setpoints with the drives disabled.
	void doDisable();
	void beginStop(const char* reason);
	//! Fastest measured hub, rad/s; +inf when a reading is not finite.
	double maxHubSpeed() const;
	bool wheelSetpointsZero() const;
	//! The target this cycle applies: zero once the command is stale or a
	//! stop is under way.
	void updateCycleTarget(double now);
	void doWheelRecovery(unsigned int wheel);
	void logRecoveryEvents(unsigned int wheel, unsigned int events);
	bool wheelLinkUp(unsigned int wheel);
	bool anyWheelHolding() const;
	bool stepStateMachine();
	void recordBlackBoxSample();
	// Wall time since construction; only its steps feed the recovery clock.
	virtual double nowMs() const;
	void advanceRecoveryClock();
	bool anyWheelFailed() const;
	void applyWheelEnable(unsigned int wheel);

	bool hasWheelStatusEnabled(unsigned int wheel);
	bool hasWheelStatusError(unsigned int wheel);
	virtual void updateStatusError();

	volatile DriverState state;
	std::ofstream logfile;
	bool canChangeActive;
	bool showedMessageChangeActive;
	int stepCount;

	ec_slavet* ecx_slaves = nullptr;
	ecx_contextt* ecx_contextp = nullptr;
	int ecx_slavecount;

	std::vector<EtherCATModule*> modules;

	std::vector<txpdo1_t> processData;
	std::vector<txpdo1_t> lastProcessData;
	long unsigned int current_ts;
	std::vector<long unsigned int> wheel_setpoint_ts;
	std::vector<long unsigned int> wheel_sensor_ts;
	unsigned int swErrorCount;
	double curr_setpoint1, curr_setpoint2;
	std::vector<std::vector<double> > prev_encoder;
	std::vector<std::vector<double> > sum_encoder;
	std::vector<std::vector<double> > abs_sum_encoder;
	bool encoderInitialized;
	double encCalibrationTolerance;
	volatile bool ethercatWkcError;
	volatile bool flagReconnectSlave;
	// Effective enable per wheel, written only by the EtherCAT thread (step).
	std::vector<bool> wheelEnabled;
	// setWheelsEnable runs on the ROS thread, so the operator's intent is
	// atomic and kept apart from what recovery allows.
	std::unique_ptr<std::atomic<bool>[]> operatorEnable;
	std::vector<bool> recoveryAllowsEnable;
	
	int firstWheel, nWheels;
	std::vector<WheelConfig> wheelConfigs;
	std::vector<WheelData> wheelData;

	double maxvlin;
	double maxva;
	double maxvlinacc;
	double maxangleacc;
	double maxvaacc;

	double wheelsetpointmin;
	double wheelsetpointmax;

	volatile bool statusError;
	volatile bool timestampError;
	
	int initTolerance;
	int initCounter;

	// INIT: bounded re-runs of the disable-then-enable start sequence.
	int initAttemptStartStep;
	unsigned int initResets;
	unsigned int maxInitResets;
	unsigned int initTimeoutSteps;

	// Recovery, one state machine per wheel. The clock is monotonic: chrony
	// steps the wall clock on this host.
	using RecoveryClock = std::chrono::steady_clock;
	RecoveryClock::time_point clockStart;
	// Advances by at most a few ms per step, so a stalled loop (a reinitialisation
	// blocks it for seconds) does not age a link loss into a give-up.
	double recoveryClockMs = 0.0;
	double lastRealMs = 0.0;
	WheelRecoveryConfig recoveryConfig;
	std::vector<WheelRecoveryMachine> recoveryMachines;
	std::vector<LinkMonitor> linkMonitors;
	// A wheel that is not in normal operation makes the platform hold: every
	// wheel's setpoint is ramped to zero until it is.
	std::vector<bool> wheelHolding;
	SetpointVectorSlew setpointSlew;
	std::vector<float> wheelSetpoints;  // setpoint1, setpoint2 per wheel
	float releaseStep;
	EthercatBlackBox* blackBox = nullptr;

	// Commands cross from the ROS thread through the mailbox; the cycle owns
	// the rest.
	CommandMailbox commandMailbox;
	StampedCommand cycleCommand;
	std::atomic<double> commandTimeoutMs{200.0};
	bool commandStale = false;
	double lastControlMs = NAN;
	double maxvlindec = 0.8;
	double maxvadec = 0.8;
	double stopVlinDec = 0.8;
	double stopVaDec = 0.8;
	std::atomic<bool> stopRequestedFlag{false};
	StopSequence stopSequence;

private:
	PlatformDriver(const PlatformDriver&);
	VelocityPlatformController velocityPlatformController;
};

} //namespace kelo

#endif //PLATFORM_DRIVER_H_
