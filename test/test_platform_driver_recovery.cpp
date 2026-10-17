#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "fake_kelo_bus.h"
#include "kelo_tulip/PlatformDriver.h"
#include "kelo_tulip/RtLog.h"

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
			driver->setTargetVelocity(commandVx, 0.0, 0.0);
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
	double commandVx = 0.3;
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

TEST_F(PlatformDriverRecovery, aDriverStartedWithTheEmergencyPairOpenWaitsInsteadOfGivingUp) {
	for (int w = 0; w < kWheels; w++)
		bus.openPair(w);
	driver->setCanChangeActive();
	EXPECT_TRUE(run(60000)) << "an E-stop held at start must not stop the driver";
	commanding = true;
	ASSERT_TRUE(run(100));

	for (int w = 0; w < kWheels; w++)
		bus.closePair(w);
	ASSERT_TRUE(run(100)) << "the start sequence re-runs as soon as the pair closes";
	for (int w = 0; w < kWheels; w++) {
		EXPECT_TRUE(wheelEnabled(w)) << "wheel " << w;
		EXPECT_EQ(bus.report(w).status1, 63) << "wheel " << w;
	}
	ASSERT_TRUE(run(300));
	EXPECT_FLOAT_EQ(speed(0), 0.0f) << "a command already there when the pair closed must not drive";
	commandVx = 0.0;
	ASSERT_TRUE(run(20));
	commandVx = 0.3;
	ASSERT_TRUE(run(300));
	EXPECT_GT(speed(0), 0.1f) << "a command after a zero one drives";
}

TEST_F(PlatformDriverRecovery, waitingForTheEmergencyPairAtStartLogsOnceNotEveryRetest) {
	kelo::RtLog& log = kelo::RtLog::instance();
	log.drain();
	const std::uint64_t droppedBefore = log.dropped();
	for (int w = 0; w < kWheels; w++)
		bus.openPair(w);
	driver->setCanChangeActive();

	std::size_t lines = 0;
	for (int second = 0; second < 60; second++) {
		ASSERT_TRUE(run(1000));
		lines += log.drain();
	}
	lines += static_cast<std::size_t>(log.dropped() - droppedBefore);
	EXPECT_LT(lines, 10u) << "an E-stop held for a minute must not flood the journal";
}

TEST_F(PlatformDriverRecovery, aDriveThatNeverBecomesReadyAtStartStillStopsTheDriver) {
	bus.setFault(1, 0x80);
	driver->setCanChangeActive();
	EXPECT_FALSE(run(4000));
}

TEST_F(PlatformDriverRecovery, anEmergencyStopWhileDrivingHoldsTheDriverInsteadOfStoppingIt) {
	startDriving();
	for (int w = 0; w < kWheels; w++)
		bus.openPair(w);
	EXPECT_TRUE(run(30000)) << "an E-stop must not stop the driver";
	for (int w = 0; w < kWheels; w++)
		EXPECT_FLOAT_EQ(speed(w), 0.0f) << "wheel " << w;

	for (int w = 0; w < kWheels; w++)
		bus.closePair(w);
	ASSERT_TRUE(run(1500));
	for (int w = 0; w < kWheels; w++) {
		EXPECT_TRUE(wheelEnabled(w)) << "wheel " << w;
		EXPECT_EQ(bus.report(w).status1, 63) << "wheel " << w;
	}
	// Released with the command still streaming: the base must not restart by itself.
	ASSERT_TRUE(run(3000));
	for (int w = 0; w < kWheels; w++)
		EXPECT_FLOAT_EQ(speed(w), 0.0f) << "wheel " << w;

	commandVx = 0.0;
	ASSERT_TRUE(run(20));
	commandVx = 0.3;
	ASSERT_TRUE(run(300));
	EXPECT_GT(speed(0), 0.1f) << "a command after a zero one drives";
}

TEST_F(PlatformDriverRecovery, aCommandTimeoutAfterTheEmergencyStopCountsAsTheZeroCommand) {
	startDriving();
	for (int w = 0; w < kWheels; w++)
		bus.openPair(w);
	ASSERT_TRUE(run(1000));
	commanding = false;
	ASSERT_TRUE(run(500));
	for (int w = 0; w < kWheels; w++)
		bus.closePair(w);
	ASSERT_TRUE(run(500));
	commanding = true;
	ASSERT_TRUE(run(300));
	EXPECT_GT(speed(0), 0.1f);
}

TEST_F(PlatformDriverRecovery, aShortPressStillNeedsAZeroCommandBeforeTheBaseDrivesAgain) {
	startDriving();
	for (int w = 0; w < kWheels; w++)
		bus.openPair(w);
	ASSERT_TRUE(run(50));
	for (int w = 0; w < kWheels; w++)
		bus.closePair(w);
	ASSERT_TRUE(run(3000));
	for (int w = 0; w < kWheels; w++) {
		EXPECT_TRUE(wheelEnabled(w)) << "wheel " << w;
		EXPECT_FLOAT_EQ(speed(w), 0.0f) << "wheel " << w;
	}
}

TEST_F(PlatformDriverRecovery, aChatteringEmergencyPairDoesNotFloodTheJournal) {
	startDriving();
	kelo::RtLog& log = kelo::RtLog::instance();
	log.drain();
	const std::uint64_t droppedBefore = log.dropped();

	std::size_t lines = 0;
	for (int bounce = 0; bounce < 60; bounce++) {
		for (int w = 0; w < kWheels; w++)
			bus.openPair(w);
		ASSERT_TRUE(run(20));
		for (int w = 0; w < kWheels; w++)
			bus.closePair(w);
		ASSERT_TRUE(run(480));
		lines += log.drain();
	}
	lines += static_cast<std::size_t>(log.dropped() - droppedBefore);
	EXPECT_LT(lines, 40u) << "60 bounces in 30 s";
}

TEST_F(PlatformDriverRecovery, oneDriveInSafeTorqueOffRampsEveryWheelToZero) {
	startDriving();
	const float before = speed(0);
	bus.openPair(2);
	ASSERT_TRUE(run(20));
	EXPECT_LT(speed(0), before) << "must already be slowing";
	EXPECT_GT(speed(0), 0.0f) << "a ramp, not an instantaneous zero";

	ASSERT_TRUE(run(10000));
	for (int w = 0; w < kWheels; w++)
		EXPECT_FLOAT_EQ(speed(w), 0.0f) << "wheel " << w;
}

TEST_F(PlatformDriverRecovery, aWheelTheOperatorSwitchedOffIsNotRecoveredWhenItsSlaveDrops) {
	startDriving();
	driver->setWheelsEnable({1, 1, 1, 0});
	ASSERT_TRUE(run(50));

	bus.loseSlave(3);
	EXPECT_TRUE(run(6000)) << "a wheel nobody wants must not stop the driver";
}

}  // namespace
