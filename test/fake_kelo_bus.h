#ifndef KELOTULIP_TEST_FAKE_KELO_BUS_H
#define KELOTULIP_TEST_FAKE_KELO_BUS_H

#include <cstring>
#include <vector>

#include "kelo_tulip/PlatformDriver.h"

namespace kelo_test {

constexpr int kWheels = 4;
constexpr double kCycleMs = 1.0;

// A driver on the test's clock: recovery, ramp, command watchdog and stop all
// read it, so seconds pass in microseconds.
class ClockedDriver : public kelo::PlatformDriver {
public:
	ClockedDriver(const std::vector<kelo::WheelConfig>& configs, const std::vector<kelo::WheelData>& data)
		: kelo::PlatformDriver(configs, data) {}

	double nowMs() const override { return clockMs; }
	double clockMs = 0.0;
};

// Four KELOdrives as the driver sees them: a process image each, and a model
// of what the drive reports back for what it was commanded.
class FakeBus {
public:
	FakeBus() {
		std::memset(slaves_, 0, sizeof(slaves_));
		std::memset(inputs_, 0, sizeof(inputs_));
		std::memset(outputs_, 0, sizeof(outputs_));
		for (int i = 1; i <= kWheels; i++) {
			slaves_[i].inputs = reinterpret_cast<uint8*>(&inputs_[i]);
			slaves_[i].outputs = reinterpret_cast<uint8*>(&outputs_[i]);
			slaves_[i].state = EC_STATE_OPERATIONAL;
			slaves_[i].islost = FALSE;
		}
	}

	ec_slavet* slaves() { return slaves_; }
	const rxpdo1_t& command(int wheel) const { return outputs_[wheel + 1]; }
	txpdo1_t& report(int wheel) { return inputs_[wheel + 1]; }

	void loseSlave(int wheel) {
		slaves_[wheel + 1].islost = TRUE;
		lost_[wheel] = true;
		std::memset(&inputs_[wheel + 1], 0, sizeof(txpdo1_t));
	}
	// Back, as observed after a dropout: motor 2 not enabled until the drive
	// has been disabled and enabled again.
	void returnSlave(int wheel, bool comesBackStuck) {
		slaves_[wheel + 1].islost = FALSE;
		lost_[wheel] = false;
		stuck_[wheel] = comesBackStuck;
		sawDisable_[wheel] = false;
	}
	void setState(int wheel, int state) { slaves_[wheel + 1].state = static_cast<uint16>(state); }
	void setFault(int wheel, uint16_t extraBits) { faultBits_[wheel] = extraBits; }
	// A hub turning faster than commanded (pushed, or still braking).
	void setExtraSpeed(int wheel, float radPerSec) { extraSpeed_[wheel] = radPerSec; }
	void setCoastSpeed(int wheel, float radPerSec) { coastSpeed_[wheel] = radPerSec; }

	// What each drive reports after the driver's last command.
	void update() {
		for (int w = 0; w < kWheels; w++) {
			if (lost_[w])
				continue;
			const bool enabled = (outputs_[w + 1].command1 & (COM1_ENABLE1 | COM1_ENABLE2)) != 0;
			if (!enabled)
				sawDisable_[w] = true;
			if (stuck_[w] && enabled && sawDisable_[w])
				stuck_[w] = false;
			txpdo1_t& in = inputs_[w + 1];
			in.status2 = 2051;
			if (stuck_[w])
				in.status1 = 61;
			else
				in.status1 = static_cast<uint16_t>((enabled ? 63 : 60) | faultBits_[w]);
			in.voltage_bus = 51.5f;
			// The hubs follow an enabled drive's setpoint; a disabled one
			// reports what it was told to coast at.
			const rxpdo1_t& out = outputs_[w + 1];
			in.velocity_1 = enabled ? out.setpoint1 + extraSpeed_[w] : coastSpeed_[w];
			in.velocity_2 = enabled ? out.setpoint2 + extraSpeed_[w] : coastSpeed_[w];
		}
	}

private:
	ec_slavet slaves_[kWheels + 1];
	txpdo1_t inputs_[kWheels + 1];
	rxpdo1_t outputs_[kWheels + 1];
	bool lost_[kWheels] = {};
	bool stuck_[kWheels] = {};
	bool sawDisable_[kWheels] = {};
	uint16_t faultBits_[kWheels] = {};
	float extraSpeed_[kWheels] = {};
	float coastSpeed_[kWheels] = {};
};

inline std::vector<kelo::WheelConfig> fakeWheelConfigs() {
	std::vector<kelo::WheelConfig> configs;
	for (int i = 0; i < kWheels; i++) {
		kelo::WheelConfig c{};
		c.ethercatNumber = i + 1;
		c.x = (i < 2) ? 0.5 : -0.5;
		c.y = (i % 2) ? 0.25 : -0.25;
		c.a = 0.0;
		c.enable = true;
		c.reverseVelocity = false;
		configs.push_back(c);
	}
	return configs;
}

inline std::vector<kelo::WheelData> fakeWheelData() {
	return std::vector<kelo::WheelData>(kWheels, kelo::WheelData{true, false, false});
}

}  // namespace kelo_test

#endif  // KELOTULIP_TEST_FAKE_KELO_BUS_H
