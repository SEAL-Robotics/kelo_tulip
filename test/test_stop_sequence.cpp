#include <gtest/gtest.h>

#include <limits>

#include "kelo_tulip/StopSequence.h"

using kelo::StopPhase;
using kelo::StopSequence;

namespace {

constexpr double kStill = 0.0;
constexpr double kMoving = 5.0;

// Steps once per ms from `from` until `phase` is reached; returns the time.
double runUntil(StopSequence& seq, double from, StopPhase phase, bool atRest, double speed, double limitMs = 10000) {
	for (double t = from; t < from + limitMs; t += 1.0)
		if (seq.step(t, atRest, speed) == phase)
			return t;
	return -1.0;
}

}  // namespace

TEST(StopSequence, isIdleUntilRequested) {
	StopSequence seq;
	EXPECT_FALSE(seq.requested());
	EXPECT_EQ(seq.step(10.0, true, kStill), StopPhase::Running);
	EXPECT_FALSE(seq.drivesEnabled());
}

TEST(StopSequence, aMovingBaseIsRampedThenSettledThenDisabled) {
	StopSequence seq;
	seq.request(0.0, true);
	EXPECT_EQ(seq.phase(), StopPhase::Ramping);
	EXPECT_TRUE(seq.drivesEnabled());

	// Setpoints still ramping: stays in Ramping however long it takes.
	EXPECT_EQ(seq.step(500.0, false, kMoving), StopPhase::Ramping);

	EXPECT_EQ(seq.step(600.0, true, kMoving), StopPhase::Settling);
	EXPECT_TRUE(seq.drivesEnabled());

	// Zero is held at least minSettleMs and until the hubs have been still.
	EXPECT_EQ(seq.step(650.0, true, kStill), StopPhase::Settling);
	const double disabledAt = runUntil(seq, 651.0, StopPhase::Disabling, true, kStill);
	EXPECT_DOUBLE_EQ(disabledAt, 700.0);
	EXPECT_FALSE(seq.drivesEnabled());
	EXPECT_FALSE(seq.timedOut());

	const double doneAt = runUntil(seq, 701.0, StopPhase::Done, true, kStill);
	EXPECT_DOUBLE_EQ(doneAt, 720.0);
}

TEST(StopSequence, theDrivesStayEnabledWhileAHubStillTurns) {
	StopSequence seq;
	seq.request(0.0, true);
	seq.step(1.0, true, kMoving);
	EXPECT_EQ(runUntil(seq, 2.0, StopPhase::Disabling, true, kMoving, 2000.0), -1.0);
	EXPECT_EQ(seq.phase(), StopPhase::Settling);
}

TEST(StopSequence, theStillWindowRestartsWhenAHubMovesAgain) {
	kelo::StopConfig config;
	config.minSettleMs = 0.0;
	StopSequence seq(config);
	seq.request(0.0, true);
	seq.step(0.0, true, kStill);
	seq.step(40.0, true, kStill);
	seq.step(41.0, true, kMoving);
	EXPECT_EQ(seq.step(60.0, true, kStill), StopPhase::Settling);
	EXPECT_EQ(seq.step(109.0, true, kStill), StopPhase::Settling);
	EXPECT_EQ(seq.step(110.0, true, kStill), StopPhase::Disabling);
}

TEST(StopSequence, theDeadlineBoundsTheRamp) {
	kelo::StopConfig config;
	config.timeoutMs = 1500.0;
	StopSequence seq(config);
	seq.request(100.0, true);
	EXPECT_EQ(seq.step(1599.0, false, kMoving), StopPhase::Ramping);
	EXPECT_EQ(seq.step(1600.0, false, kMoving), StopPhase::Disabling);
	EXPECT_TRUE(seq.timedOut());
}

TEST(StopSequence, theDeadlineBoundsTheSettling) {
	kelo::StopConfig config;
	config.timeoutMs = 1000.0;
	StopSequence seq(config);
	seq.request(0.0, true);
	seq.step(10.0, true, kMoving);
	EXPECT_EQ(seq.step(999.0, true, kMoving), StopPhase::Settling);
	EXPECT_EQ(seq.step(1000.0, true, kMoving), StopPhase::Disabling);
	EXPECT_TRUE(seq.timedOut());
}

TEST(StopSequence, aNonFiniteHubSpeedNeverCountsAsStill) {
	StopSequence seq;
	seq.request(0.0, true);
	seq.step(1.0, true, kStill);
	const double nan = std::numeric_limits<double>::quiet_NaN();
	EXPECT_EQ(runUntil(seq, 2.0, StopPhase::Disabling, true, nan, 2000.0), -1.0);
	EXPECT_EQ(seq.step(3000.0, true, nan), StopPhase::Disabling);
	EXPECT_TRUE(seq.timedOut());
}

TEST(StopSequence, aClockThatReadsNaNEndsTheStopRatherThanHangingIt) {
	StopSequence seq;
	seq.request(0.0, true);
	EXPECT_EQ(seq.step(std::numeric_limits<double>::quiet_NaN(), false, kMoving), StopPhase::Disabling);
}

TEST(StopSequence, aBaseThatNeverDroveIsDisabledAtOnce) {
	StopSequence seq;
	seq.request(0.0, false);
	EXPECT_EQ(seq.phase(), StopPhase::Disabling);
	EXPECT_FALSE(seq.drivesEnabled());
	EXPECT_EQ(runUntil(seq, 1.0, StopPhase::Done, true, kStill), 20.0);
}

TEST(StopSequence, aSecondRequestDoesNotRestartTheDeadline) {
	kelo::StopConfig config;
	config.timeoutMs = 1000.0;
	StopSequence seq(config);
	seq.request(0.0, true);
	seq.request(900.0, true);
	EXPECT_EQ(seq.step(1000.0, false, kMoving), StopPhase::Disabling);
}

TEST(StopSequence, doneIsFinal) {
	StopSequence seq;
	seq.request(0.0, false);
	runUntil(seq, 1.0, StopPhase::Done, true, kStill);
	EXPECT_EQ(seq.step(100.0, false, kMoving), StopPhase::Done);
	seq.request(200.0, true);
	EXPECT_EQ(seq.phase(), StopPhase::Done);
}
