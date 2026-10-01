#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "fake_kelo_bus.h"
#include "kelo_tulip/PlatformDriver.h"

namespace {

using kelo_test::ClockedDriver;
using kelo_test::FakeBus;
using kelo_test::kCycleMs;
using kelo_test::kWheels;

constexpr double kVx = 0.3;  // m/s

class PlatformDriverStop : public ::testing::Test {
protected:
	void SetUp() override {
		driver = std::make_unique<ClockedDriver>(kelo_test::fakeWheelConfigs(), kelo_test::fakeWheelData());
		driver->setMaxvlin(1.0);
		driver->setMaxvlinacc(1.0e6);
		// 1 m/s^2: the ramp down is long enough to observe.
		driver->setMaxvlindec(1.0);
		driver->setMaxvadec(1.0);
		driver->setStopParameters(1.0, 1.0, 3.0);
		ASSERT_TRUE(driver->initEtherCAT(bus.slaves(), kWheels));
		bus.update();
	}

	bool cycle() {
		driver->clockMs += kCycleMs;
		if (commanding)
			driver->setTargetVelocity(vx, 0.0, 0.0);
		const bool keepRunning = driver->step();
		bus.update();
		return keepRunning;
	}

	bool run(double ms) {
		for (int i = 0; i < static_cast<int>(ms / kCycleMs); i++)
			if (!cycle())
				return false;
		return true;
	}

	void startDriving() {
		driver->setCanChangeActive();
		ASSERT_TRUE(run(200));
		commanding = true;
		ASSERT_TRUE(run(50));
		ASSERT_GT(speed(0), 1.0f);
	}

	bool enabled(int wheel) const {
		return (bus.command(wheel).command1 & (COM1_ENABLE1 | COM1_ENABLE2)) != 0;
	}

	bool anyEnabled() const {
		for (int w = 0; w < kWheels; w++)
			if (enabled(w))
				return true;
		return false;
	}

	float speed(int wheel) const {
		return std::fabs(bus.command(wheel).setpoint1) + std::fabs(bus.command(wheel).setpoint2);
	}

	float maxSpeed() const {
		float fastest = 0.0f;
		for (int w = 0; w < kWheels; w++)
			fastest = std::max(fastest, speed(w));
		return fastest;
	}

	bool setpointsFinite() const {
		for (int w = 0; w < kWheels; w++)
			if (!std::isfinite(bus.command(w).setpoint1) || !std::isfinite(bus.command(w).setpoint2))
				return false;
		return true;
	}

	// What the stop did, cycle by cycle, until the driver ended its loop.
	struct StopTrace {
		int cycles = 0;
		int firstZeroCycle = -1;      // every setpoint at zero
		int firstDisabledCycle = -1;  // every drive commanded disabled
		float largestRise = 0.0f;
		bool enabledFromZeroToDisable = true;
		bool ended = false;
	};

	StopTrace stopAndTrace(int maxCycles = 10000) {
		StopTrace t;
		float previous = maxSpeed();
		driver->requestStop();
		for (; t.cycles < maxCycles; t.cycles++) {
			const bool keepRunning = cycle();
			const float now = maxSpeed();
			t.largestRise = std::max(t.largestRise, now - previous);
			previous = now;
			if (t.firstZeroCycle < 0 && now == 0.0f)
				t.firstZeroCycle = t.cycles;
			if (t.firstDisabledCycle < 0 && !anyEnabled())
				t.firstDisabledCycle = t.cycles;
			if (t.firstZeroCycle >= 0 && t.firstDisabledCycle < 0 && !anyEnabled())
				t.enabledFromZeroToDisable = false;
			if (!keepRunning) {
				t.ended = true;
				break;
			}
		}
		return t;
	}

	FakeBus bus;
	std::unique_ptr<ClockedDriver> driver;
	bool commanding = false;
	double vx = kVx;
};

TEST_F(PlatformDriverStop, aStopWhileDrivingRampsToZeroBeforeTheDrivesAreReleased) {
	startDriving();
	const StopTrace t = stopAndTrace();

	ASSERT_TRUE(t.ended);
	EXPECT_LE(t.largestRise, 1e-4f) << "the setpoints only ever go down";
	// 0.3 m/s at 1 m/s^2 is 300 ms of ramp: never a cut.
	EXPECT_GE(t.firstZeroCycle, 280);
	EXPECT_LE(t.firstZeroCycle, 320);
	// Zero is held with the drives enabled before they are disabled.
	EXPECT_GE(t.firstDisabledCycle, t.firstZeroCycle + 100);
	EXPECT_LE(t.firstDisabledCycle, t.firstZeroCycle + 200);
	// The loop ends after the disable frames, well before the deadline.
	EXPECT_GE(t.cycles, t.firstDisabledCycle + 19);
	EXPECT_LT(t.cycles, 3000);
	EXPECT_FALSE(anyEnabled());
	EXPECT_EQ(maxSpeed(), 0.0f);
}

TEST_F(PlatformDriverStop, theStopIsARampNotACut) {
	startDriving();
	const float before = maxSpeed();
	driver->requestStop();
	ASSERT_TRUE(run(20));
	EXPECT_LT(maxSpeed(), before);
	EXPECT_GT(maxSpeed(), 0.8f * before);
	EXPECT_TRUE(anyEnabled());
}

TEST_F(PlatformDriverStop, theStopRampNeverBrakesHarderThanTheConfiguredDeceleration) {
	driver->setStopParameters(1000.0, 1000.0, 3.0);
	startDriving();
	const StopTrace t = stopAndTrace();
	EXPECT_GE(t.firstZeroCycle, 280) << "vlin_dec_max (1 m/s^2) still bounds the stop";
}

TEST_F(PlatformDriverStop, theStopDecelerationAppliesWhenItIsGentler) {
	driver->setStopParameters(0.5, 0.5, 3.0);
	startDriving();
	const StopTrace t = stopAndTrace();
	EXPECT_GE(t.firstZeroCycle, 580);
	EXPECT_LE(t.firstZeroCycle, 620);
}

TEST_F(PlatformDriverStop, commandsAfterTheStopRequestAreIgnored) {
	startDriving();
	vx = 0.9;  // the source keeps commanding, faster
	const StopTrace t = stopAndTrace();
	ASSERT_TRUE(t.ended);
	EXPECT_LE(t.largestRise, 1e-4f);
}

TEST_F(PlatformDriverStop, aHubStillTurningKeepsTheDrivesEnabledUntilTheDeadline) {
	startDriving();
	bus.setExtraSpeed(2, 5.0f);
	const StopTrace t = stopAndTrace();
	ASSERT_TRUE(t.ended);
	EXPECT_GE(t.firstDisabledCycle, 2995);
	EXPECT_LE(t.firstDisabledCycle, 3005);
	EXPECT_TRUE(t.enabledFromZeroToDisable);
}

TEST_F(PlatformDriverStop, aStopBeforeTheBaseEverDroveDisablesAtOnce) {
	ASSERT_TRUE(run(100));  // INIT/READY, never ACTIVE
	const StopTrace t = stopAndTrace();
	ASSERT_TRUE(t.ended);
	EXPECT_EQ(t.firstDisabledCycle, 0);
	EXPECT_LE(t.cycles, 25);
}

TEST_F(PlatformDriverStop, aWheelThatCannotBeRecoveredRampsTheOthersBeforeTheLoopEnds) {
	startDriving();
	bus.setFault(1, 0x80);
	bool running = true;
	int cycles = 0;
	while (running && cycles < 10000) {
		running = cycle();
		cycles++;
	}
	ASSERT_FALSE(running);
	EXPECT_FALSE(anyEnabled()) << "the loop ends only after every drive was disabled";
	EXPECT_EQ(maxSpeed(), 0.0f);
}

// The command watchdog runs in the EtherCAT cycle: nothing else is stepped
// here, as when the ROS executor is stalled.
TEST_F(PlatformDriverStop, aSilentCommandSourceIsRampedToZeroByTheCycleAlone) {
	startDriving();
	const float before = maxSpeed();
	commanding = false;

	ASSERT_TRUE(run(190));
	EXPECT_FLOAT_EQ(maxSpeed(), before) << "inside the 200 ms timeout the command holds";

	ASSERT_TRUE(run(30));
	EXPECT_LT(maxSpeed(), before) << "past the timeout it ramps down";
	EXPECT_GT(maxSpeed(), 0.0f) << "a ramp, not a cut";

	ASSERT_TRUE(run(400));
	EXPECT_EQ(maxSpeed(), 0.0f);
	EXPECT_TRUE(anyEnabled()) << "a timeout stops the base; it does not release it";
}

TEST_F(PlatformDriverStop, commandsResumeAfterATimeout) {
	startDriving();
	commanding = false;
	ASSERT_TRUE(run(700));
	ASSERT_EQ(maxSpeed(), 0.0f);
	commanding = true;
	ASSERT_TRUE(run(50));
	EXPECT_GT(maxSpeed(), 1.0f);
}

TEST_F(PlatformDriverStop, theCommandTimeoutIsConfigurable) {
	driver->setCommandTimeout(0.05);
	startDriving();
	const float before = maxSpeed();
	commanding = false;
	ASSERT_TRUE(run(40));
	EXPECT_FLOAT_EQ(maxSpeed(), before);
	ASSERT_TRUE(run(20));
	EXPECT_LT(maxSpeed(), before);
}

TEST_F(PlatformDriverStop, anInvalidCommandTimeoutKeepsTheDefault) {
	for (const double bad : {0.0, -1.0, 100.0, std::numeric_limits<double>::quiet_NaN()})
		driver->setCommandTimeout(bad);
	startDriving();
	const float before = maxSpeed();
	commanding = false;
	ASSERT_TRUE(run(190));
	EXPECT_FLOAT_EQ(maxSpeed(), before);
	ASSERT_TRUE(run(30));
	EXPECT_LT(maxSpeed(), before);
}

TEST_F(PlatformDriverStop, aNonFiniteCommandIsAStopAndNeverReachesTheDrives) {
	startDriving();
	for (const double bad : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
		vx = bad;
		for (int i = 0; i < 600; i++) {
			ASSERT_TRUE(cycle());
			ASSERT_TRUE(setpointsFinite());
		}
		EXPECT_EQ(maxSpeed(), 0.0f);
		vx = kVx;
		ASSERT_TRUE(run(50));
		EXPECT_GT(maxSpeed(), 1.0f) << "a finite command afterwards drives again";
	}
}

TEST_F(PlatformDriverStop, aNonFinitePivotReadingNeverReachesTheDrivesAsNaN) {
	startDriving();
	for (int w = 0; w < kWheels; w++)
		bus.report(w).encoder_pivot = std::numeric_limits<float>::quiet_NaN();
	for (int i = 0; i < 100; i++) {
		ASSERT_TRUE(cycle());
		ASSERT_TRUE(setpointsFinite());
	}
}

TEST_F(PlatformDriverStop, aClockJumpForwardDoesNotStepTheSetpoints) {
	driver->setMaxvlinacc(1.0);
	driver->setCanChangeActive();
	ASSERT_TRUE(run(200));
	vx = 1.0;
	commanding = true;
	ASSERT_TRUE(run(100));
	const float before = maxSpeed();

	driver->clockMs += 3600.0 * 1000.0;
	ASSERT_TRUE(cycle());

	// Acceleration sees at most MAX_ACCEL_DT_SEC: 0.01 m/s at 1 m/s^2.
	const float hubStep = 2.0f * 0.01f / 0.0525f;
	EXPECT_LE(maxSpeed() - before, hubStep + 1e-3f);
}

TEST_F(PlatformDriverStop, aClockThatStandsStillRampsByNothing) {
	driver->setMaxvlinacc(1.0);
	driver->setCanChangeActive();
	ASSERT_TRUE(run(200));
	vx = 1.0;
	commanding = true;
	ASSERT_TRUE(run(100));
	const float before = maxSpeed();

	for (int i = 0; i < 100; i++) {
		driver->setTargetVelocity(vx, 0.0, 0.0);
		ASSERT_TRUE(driver->step());
		bus.update();
	}
	EXPECT_FLOAT_EQ(maxSpeed(), before);
}

}  // namespace
