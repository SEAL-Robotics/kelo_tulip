#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "fake_kelo_bus.h"
#include "kelo_tulip/PlatformDriver.h"

namespace {

using kelo_test::ClockedDriver;
using kelo_test::FakeBus;
using kelo_test::kCycleMs;
using kelo_test::kWheels;

class PlatformDriverRecovery : public ::testing::Test {
protected:
	void SetUp() override {
		driver = std::make_unique<ClockedDriver>(kelo_test::fakeWheelConfigs(), kelo_test::fakeWheelData());
		driver->setMaxvlin(1.0);
		driver->setMaxvlinacc(1.0e6);
		driver->setMaxvlindec(1.0e6);
		ASSERT_TRUE(driver->initEtherCAT(bus.slaves(), kWheels));
		bus.update();
	}

	// One EtherCAT cycle; false once the driver asked to stop.
	bool cycle() {
		driver->clockMs += kCycleMs;
		// A live command source, as the cycle's command watchdog expects.
		if (commanding)
			driver->setTargetVelocity(0.3, 0.0, 0.0);
		const bool keepRunning = driver->step();
		bus.update();
		return keepRunning;
	}

	// Cycles for `ms`, stopping at the first refusal.
	bool run(double ms) {
		const int cycles = static_cast<int>(ms / kCycleMs);
		for (int i = 0; i < cycles; i++)
			if (!cycle())
				return false;
		return true;
	}

	void startDriving() {
		driver->setCanChangeActive();
		ASSERT_TRUE(run(200));
		commanding = true;
		ASSERT_TRUE(run(50));
	}

	bool wheelEnabled(int wheel) const {
		return (bus.command(wheel).command1 & (COM1_ENABLE1 | COM1_ENABLE2)) == (COM1_ENABLE1 | COM1_ENABLE2);
	}

	float speed(int wheel) const {
		return std::fabs(bus.command(wheel).setpoint1) + std::fabs(bus.command(wheel).setpoint2);
	}

	FakeBus bus;
	std::unique_ptr<ClockedDriver> driver;
	bool commanding = false;
};

TEST_F(PlatformDriverRecovery, drivesWhenAllWheelsAreHealthy) {
	startDriving();
	for (int w = 0; w < kWheels; w++) {
		EXPECT_TRUE(wheelEnabled(w)) << "wheel " << w;
		EXPECT_GT(speed(w), 0.1f) << "wheel " << w;
	}
}

TEST_F(PlatformDriverRecovery, aSlaveGoneForTwoSecondsThatReturnsDisabledIsReenabledWithoutStoppingTheDriver) {
	startDriving();

	bus.loseSlave(3);
	EXPECT_TRUE(run(1900)) << "the driver must wait for the slave, not give up";
	bus.returnSlave(3, true);

	EXPECT_TRUE(run(1500)) << "the driver must re-enable the returned slave";
	EXPECT_TRUE(wheelEnabled(3));
	EXPECT_EQ(bus.report(3).status1, 63);
}

TEST_F(PlatformDriverRecovery, aStallOfTheLoopAfterTheSlaveReturnedDoesNotCountAsTheSlaveBeingGone) {
	startDriving();

	bus.loseSlave(3);
	ASSERT_TRUE(run(1000));
	bus.returnSlave(3, true);
	// A reinitialisation blocks the loop for seconds; no step runs meanwhile.
	driver->clockMs += 8000.0;

	EXPECT_TRUE(run(1500));
	EXPECT_TRUE(wheelEnabled(3));
}

TEST_F(PlatformDriverRecovery, aSlaveThatNeverReturnsStopsTheDriverAfterTheWindow) {
	startDriving();

	bus.loseSlave(3);

	EXPECT_FALSE(run(5500));
}

TEST_F(PlatformDriverRecovery, aSlaveThatKeepsAFaultBitStopsTheDriverAsBefore) {
	startDriving();

	bus.setFault(1, 0x80);  // an overcurrent-type bit that never clears

	EXPECT_FALSE(run(4000));
}

TEST_F(PlatformDriverRecovery, aSlaveThatDropsOutAgainAndAgainIsGivenUp) {
	startDriving();

	bool stopped = false;
	for (int dropout = 0; dropout < 30 && !stopped; dropout++) {
		bus.loseSlave(3);
		stopped = !run(300);
		if (stopped)
			break;
		bus.returnSlave(3, true);
		stopped = !run(400);
	}

	EXPECT_TRUE(stopped);
}

TEST_F(PlatformDriverRecovery, everyWheelIsRampedToZeroWhileOneIsMissing) {
	startDriving();
	const float before = speed(0);
	ASSERT_GT(before, 0.1f);

	bus.loseSlave(3);
	ASSERT_TRUE(run(20));
	const float early = speed(0);
	EXPECT_LT(early, before) << "must already be slowing";
	EXPECT_GT(early, 0.0f) << "a ramp, not an instantaneous zero";

	// Prompt: a full-speed platform is at rest well inside a second.
	ASSERT_TRUE(run(280));
	for (int w = 0; w < 3; w++)
		EXPECT_FLOAT_EQ(speed(w), 0.0f) << "wheel " << w;
	EXPECT_FALSE(wheelEnabled(3));
}

TEST_F(PlatformDriverRecovery, theWheelsResumeByRampingBackUpAfterTheRecovery) {
	startDriving();
	const float before = speed(0);
	bus.loseSlave(3);
	ASSERT_TRUE(run(1500));
	bus.returnSlave(3, true);

	// Never a step back to the live command: two setpoints, 0.05 rad/s each per cycle.
	float previous = speed(0);
	float largestStep = 0.0f;
	for (int i = 0; i < 800; i++) {
		ASSERT_TRUE(cycle());
		largestStep = std::max(largestStep, std::fabs(speed(0) - previous));
		previous = speed(0);
	}
	EXPECT_LE(largestStep, 0.11f);
	EXPECT_GT(speed(0), before * 0.9f);
}

TEST_F(PlatformDriverRecovery, aBriefNonOperationalStateFromALostFrameDoesNotSlowTheBase) {
	startDriving();
	const float before = speed(0);

	// SOEM's state read reports 0 for every slave after one lost frame.
	for (int w = 0; w < kWheels; w++)
		bus.setState(w, 0);
	ASSERT_TRUE(run(64));
	for (int w = 0; w < kWheels; w++)
		bus.setState(w, EC_STATE_OPERATIONAL);
	ASSERT_TRUE(run(20));

	EXPECT_NEAR(speed(0), before, 0.001f);
}

TEST_F(PlatformDriverRecovery, aWheelTheOperatorSwitchedOffIsNotRecoveredWhenItsSlaveDrops) {
	startDriving();
	driver->setWheelsEnable({1, 1, 1, 0});
	ASSERT_TRUE(run(50));

	bus.loseSlave(3);
	EXPECT_TRUE(run(6000)) << "a wheel nobody wants must not stop the driver";
}

}  // namespace
