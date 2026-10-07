#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "kelo_tulip/OdometryFreshness.h"

using kelo::OdometryFreshnessTracker;

namespace {

const uint64_t T0 = 5000000000ULL;
const uint64_t STEP = 50000000ULL;

TEST(OdometryFreshness, outageSequenceFlagsEveryTickWithOneTickDebounce) {
	OdometryFreshnessTracker tracker(2, 1);
	const uint64_t ts[] = {T0, T0 + STEP, T0 + STEP, T0 + STEP, 9000000000ULL};
	const bool stale[] = {false, false, true, true, false};
	const bool becameStale[] = {false, false, true, false, false};
	const bool becameFresh[] = {false, false, false, false, true};

	for (int i = 0; i < 5; i++) {
		auto u = tracker.update({ts[i], ts[i]});
		EXPECT_EQ(u.stale, stale[i]) << "tick " << i;
		EXPECT_EQ(u.becameStale, becameStale[i]) << "tick " << i;
		EXPECT_EQ(u.becameFresh, becameFresh[i]) << "tick " << i;
	}
}

TEST(OdometryFreshness, singleRepeatedTimestampIsNotStale) {
	OdometryFreshnessTracker tracker(1, 3);
	tracker.update({T0});
	tracker.update({T0 + STEP});
	EXPECT_FALSE(tracker.update({T0 + STEP}).stale);
	EXPECT_FALSE(tracker.update({T0 + STEP}).stale);
	auto u = tracker.update({T0 + STEP});
	EXPECT_TRUE(u.stale);
	EXPECT_TRUE(u.becameStale);
}

TEST(OdometryFreshness, freshSampleResetsTheDebounceCount) {
	OdometryFreshnessTracker tracker(1, 3);
	tracker.update({T0});
	tracker.update({T0});
	tracker.update({T0});
	tracker.update({T0 + STEP});
	EXPECT_FALSE(tracker.update({T0 + STEP}).stale);
	EXPECT_FALSE(tracker.update({T0 + STEP}).stale);
}

TEST(OdometryFreshness, oneStaleWheelMakesTheRobotStale) {
	OdometryFreshnessTracker tracker(2, 1);
	tracker.update({T0, T0});
	EXPECT_TRUE(tracker.update({T0 + STEP, T0}).stale);
}

TEST(OdometryFreshness, silentUntilTheFirstFreshSample) {
	OdometryFreshnessTracker tracker(1, 2);
	for (int i = 0; i < 5; i++) {
		auto u = tracker.update({0});
		EXPECT_TRUE(u.stale);
		EXPECT_FALSE(u.becameStale);
		EXPECT_FALSE(u.becameFresh);
	}
	auto first = tracker.update({T0});
	EXPECT_FALSE(first.stale);
	EXPECT_FALSE(first.becameFresh);
	EXPECT_TRUE(first.resync);
}

TEST(OdometryFreshness, resyncOnEveryStaleToFreshTransition) {
	OdometryFreshnessTracker tracker(1, 1);
	tracker.update({T0});
	tracker.update({T0});
	auto u = tracker.update({100});  // drive restarted its clock near zero
	EXPECT_FALSE(u.stale);
	EXPECT_TRUE(u.becameFresh);
	EXPECT_TRUE(u.resync);
	EXPECT_FALSE(tracker.update({100 + STEP}).resync);
}

TEST(OdometryFreshness, deltaIsTheTimestampAdvanceOfTheLastUpdate) {
	OdometryFreshnessTracker tracker(1, 3);
	tracker.update({T0});
	tracker.update({T0 + STEP});
	EXPECT_EQ(tracker.deltaNs(0), STEP);
}

TEST(OdometryFreshness, unchangedTicksCarryNoInformationButAreNotStaleYet) {
	OdometryFreshnessTracker tracker(2, 3);
	auto first = tracker.update({T0, T0});
	EXPECT_FALSE(first.noNewSample);
	for (int i = 0; i < 2; i++) {
		auto u = tracker.update({T0, T0 + STEP * (i + 1)});
		EXPECT_TRUE(u.noNewSample) << "tick " << i;
		EXPECT_FALSE(u.stale) << "tick " << i;
		EXPECT_FALSE(u.becameStale) << "tick " << i;
	}
	auto back = tracker.update({T0 + 3 * STEP, T0 + 3 * STEP});
	EXPECT_FALSE(back.noNewSample);
	EXPECT_FALSE(back.resync);
}

TEST(OdometryFreshness, holdsOdometryOnlyWhileTheSampleIsMissing) {
	EXPECT_TRUE(kelo::holdOdometry(true, false));
	EXPECT_TRUE(kelo::holdOdometry(false, true));
	EXPECT_FALSE(kelo::holdOdometry(false, false));
}

TEST(OdometryFreshness, warnsOnceWhenNoDataArrivesAfterStart) {
	OdometryFreshnessTracker tracker(1, 3, 4);
	int warnings = 0;
	for (int i = 0; i < 10; i++)
		warnings += tracker.update({0}).noDataWarning ? 1 : 0;
	EXPECT_EQ(warnings, 1);
}

TEST(OdometryFreshness, noNoDataWarningOnceDataArrived) {
	OdometryFreshnessTracker tracker(1, 3, 4);
	tracker.update({T0});
	for (int i = 0; i < 10; i++)
		EXPECT_FALSE(tracker.update({T0 + STEP * (i + 1)}).noDataWarning);
}

TEST(OdometryFreshness, deltaSpansTheWholeGapAfterHeldSteps) {
	OdometryFreshnessTracker tracker(2, 3);
	tracker.update({T0, T0});
	auto held = tracker.update({T0, T0 + STEP});
	EXPECT_TRUE(held.noNewSample);
	tracker.update({T0 + 2 * STEP, T0 + 2 * STEP});
	EXPECT_EQ(tracker.deltaNs(0), 2 * STEP);
	EXPECT_EQ(tracker.deltaNs(1), 2 * STEP);
}

const double PERIOD = 0.05;
const double NO_ALIASING_LIMIT = 1e9;

TEST(OdometryFreshness, encoderDeltaLimitCoversTheDebounceGap) {
	for (int n = 1; n <= 6; n++)
		EXPECT_DOUBLE_EQ(kelo::encoderDeltaLimitSec(n, PERIOD, NO_ALIASING_LIMIT), n * PERIOD + 0.5 * PERIOD) << "n " << n;
}

TEST(OdometryFreshness, aliasingLimitKeepsAWheelUnderPiRadiansPerGap) {
	const double radius = 0.0825;
	const double speed = 2.46;
	double limit = kelo::encoderAliasingLimitSec(radius, speed);
	EXPECT_LT(speed / radius * limit, M_PI);
	EXPECT_NEAR(limit, 0.8 * M_PI * radius / speed, 1e-12);
	EXPECT_GT(kelo::encoderAliasingLimitSec(radius, 0.0), 1e6);
}

TEST(OdometryFreshness, encoderDeltaLimitIsCappedAtTheAliasingBound) {
	const double alias = kelo::encoderAliasingLimitSec(0.0825, 2.46);
	EXPECT_DOUBLE_EQ(kelo::encoderDeltaLimitSec(3, PERIOD, alias), alias);
	EXPECT_DOUBLE_EQ(kelo::encoderDeltaLimitSec(1, PERIOD, alias), std::min(0.075, alias));
	EXPECT_LT(kelo::encoderDeltaLimitSec(3, PERIOD, alias), 0.1);
}

TEST(OdometryFreshness, fallbackIntegratesTheGapUpToTheDebounceLimit) {
	const double maxGap = 3 * PERIOD + 0.5 * PERIOD;
	EXPECT_DOUBLE_EQ(kelo::fallbackDtSec(0.15, false, maxGap, PERIOD), 0.15);
	EXPECT_DOUBLE_EQ(kelo::fallbackDtSec(maxGap, false, maxGap, PERIOD), maxGap);
	EXPECT_DOUBLE_EQ(kelo::fallbackDtSec(0.3, false, maxGap, PERIOD), PERIOD);
	EXPECT_DOUBLE_EQ(kelo::fallbackDtSec(1e10, false, maxGap, PERIOD), PERIOD);
	EXPECT_DOUBLE_EQ(kelo::fallbackDtSec(0.0, false, maxGap, PERIOD), PERIOD);
}

TEST(OdometryFreshness, resyncDtIsTheLoopPeriodWhateverTheGap) {
	const double maxGap = 3 * PERIOD + 0.5 * PERIOD;
	EXPECT_DOUBLE_EQ(kelo::fallbackDtSec(0.15, true, maxGap, PERIOD), PERIOD);
	EXPECT_DOUBLE_EQ(kelo::fallbackDtSec(5.0, true, maxGap, PERIOD), PERIOD);
}

TEST(OdometryFreshness, encoderDeltaUsability) {
	const double limit = 0.175;
	EXPECT_TRUE(kelo::isEncoderDeltaUsable(0.05, false, limit));
	EXPECT_TRUE(kelo::isEncoderDeltaUsable(0.15, false, limit));
	EXPECT_FALSE(kelo::isEncoderDeltaUsable(0.2, false, limit));
	EXPECT_FALSE(kelo::isEncoderDeltaUsable(0.0, false, limit));
	EXPECT_FALSE(kelo::isEncoderDeltaUsable(0.05, true, limit));
}

TEST(OdometryFreshness, noWheelsIsNeverStale) {
	OdometryFreshnessTracker tracker(0, 3);
	EXPECT_FALSE(tracker.update({}).stale);
}

TEST(OdometryFreshness, staleTwistCovarianceValidation) {
	EXPECT_TRUE(kelo::isValidStaleTwistCovariance(1e6));
	EXPECT_FALSE(kelo::isValidStaleTwistCovariance(0.0));
	EXPECT_FALSE(kelo::isValidStaleTwistCovariance(-1.0));
	EXPECT_FALSE(kelo::isValidStaleTwistCovariance(std::numeric_limits<double>::quiet_NaN()));
	EXPECT_FALSE(kelo::isValidStaleTwistCovariance(std::numeric_limits<double>::infinity()));
}

TEST(OdometryFreshness, odomVarianceValidation) {
	EXPECT_TRUE(kelo::isValidOdomVariance(1e-2));
	EXPECT_TRUE(kelo::isValidOdomVariance(1e3));
	EXPECT_FALSE(kelo::isValidOdomVariance(0.0));
	EXPECT_FALSE(kelo::isValidOdomVariance(-1e-2));
	EXPECT_FALSE(kelo::isValidOdomVariance(std::numeric_limits<double>::quiet_NaN()));
	EXPECT_FALSE(kelo::isValidOdomVariance(std::numeric_limits<double>::infinity()));
}

TEST(OdometryFreshness, staleTwistCovarianceOnlyWhileStale) {
	EXPECT_DOUBLE_EQ(kelo::twistCovariance(false, 1e-3, 1e6), 1e-3);
	EXPECT_DOUBLE_EQ(kelo::twistCovariance(true, 1e-3, 1e6), 1e6);
}

}  // namespace
