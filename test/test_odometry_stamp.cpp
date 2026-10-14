#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "kelo_tulip/OdometryStamp.h"

namespace {

using kelo::kNoInstantNs;
using kelo::OdometryClock;
using kelo::OdomPose;
using kelo::TwistSource;

const int64_t MS = 1000000;
const int64_t T0 = 1760000000LL * 1000000000LL;
const double WINDOW = 0.05;

TEST(IntegratePose, aStraightStepMovesAlongTheHeading) {
	const OdomPose p = kelo::integratePose(OdomPose{1.0, 2.0, M_PI / 2}, 0.4, 0.0, 0.0, 0.5);
	EXPECT_NEAR(p.x, 1.0, 1e-12);
	EXPECT_NEAR(p.y, 2.2, 1e-12);
	EXPECT_NEAR(p.a, M_PI / 2, 1e-12);
}

TEST(IntegratePose, aTurningStepMovesAlongTheChordAtTheMeanHeading) {
	const OdomPose p = kelo::integratePose(OdomPose{0.0, 0.0, 0.0}, 1.0, 0.0, 0.4, 0.5);
	EXPECT_NEAR(p.x, 0.5 * std::cos(0.1), 1e-12);
	EXPECT_NEAR(p.y, 0.5 * std::sin(0.1), 1e-12);
	EXPECT_NEAR(p.a, 0.2, 1e-12);
}

TEST(IntegratePose, theHeadingStaysWithinPlusMinusPi) {
	const OdomPose p = kelo::integratePose(OdomPose{0.0, 0.0, 3.1}, 0.0, 0.0, 1.0, 0.2);
	EXPECT_NEAR(p.a, 3.3 - 2 * M_PI, 1e-12);
}

TEST(IntegratePose, aNonFiniteHeadingIsPassedOnNotLoopedOn) {
	const OdomPose p = kelo::integratePose(OdomPose{0.0, 0.0, 0.0}, 0.0, 0.0,
		std::numeric_limits<double>::infinity(), 0.05);
	EXPECT_FALSE(std::isfinite(p.a));
}

TEST(IntegratePose, halfTheStepOfAStraightMotionIsHalfway) {
	const OdomPose start{0.3, -0.1, 0.7};
	const OdomPose end = kelo::integratePose(start, 0.8, -0.2, 0.0, WINDOW);
	const OdomPose mid = kelo::integratePose(start, 0.8, -0.2, 0.0, 0.5 * WINDOW);
	EXPECT_NEAR(mid.x, 0.5 * (start.x + end.x), 1e-12);
	EXPECT_NEAR(mid.y, 0.5 * (start.y + end.y), 1e-12);
}

TEST(TwistSource, followsHowTheStepMeasuredTheVelocity) {
	EXPECT_EQ(kelo::twistSource(false, true), TwistSource::EncoderWindow);
	EXPECT_EQ(kelo::twistSource(false, false), TwistSource::DriveVelocity);
	EXPECT_EQ(kelo::twistSource(true, true), TwistSource::Held);
	EXPECT_EQ(kelo::twistSource(true, false), TwistSource::Held);
}

TEST(CommonSampleInstant, isTheEarliestOfTheWheelsLatestSamples) {
	EXPECT_EQ(kelo::commonSampleInstantNs({T0, T0, T0, T0}), T0);
	EXPECT_EQ(kelo::commonSampleInstantNs({T0, T0 - MS, T0, T0}), T0 - MS);
}

TEST(CommonSampleInstant, aWheelWithoutAnInstantLeavesTheWindowWithoutOne) {
	EXPECT_EQ(kelo::commonSampleInstantNs({T0, kNoInstantNs, T0}), kNoInstantNs);
	EXPECT_EQ(kelo::commonSampleInstantNs({}), kNoInstantNs);
}

TEST(OdometryClock, anEncoderWindowIsStampedAtItsMidpointWithThePoseHalfwayThrough) {
	OdometryClock clock;
	const OdometryClock::Stamp s = clock.next(TwistSource::EncoderWindow, T0, WINDOW, T0 + 30 * MS);
	EXPECT_EQ(s.ns, T0 - 25 * MS);
	EXPECT_DOUBLE_EQ(s.poseFraction, 0.5);
}

TEST(OdometryClock, driveVelocitiesAreStampedAtTheSampleWithTheWholeStep) {
	OdometryClock clock;
	const OdometryClock::Stamp s = clock.next(TwistSource::DriveVelocity, T0, WINDOW, T0 + 30 * MS);
	EXPECT_EQ(s.ns, T0);
	EXPECT_DOUBLE_EQ(s.poseFraction, 1.0);
}

TEST(OdometryClock, aHeldStepKeepsTheInstantOfTheLastUsedSample) {
	OdometryClock clock;
	clock.next(TwistSource::EncoderWindow, T0, WINDOW, T0 + 30 * MS);
	const OdometryClock::Stamp held = clock.next(TwistSource::Held, T0 + 50 * MS, WINDOW, T0 + 80 * MS);
	EXPECT_EQ(held.ns, T0);
	EXPECT_EQ(clock.next(TwistSource::Held, T0 + 100 * MS, WINDOW, T0 + 130 * MS).ns, T0);
}

TEST(OdometryClock, beforeAnySampleTheRosClockStamps) {
	OdometryClock clock;
	EXPECT_EQ(clock.next(TwistSource::Held, kNoInstantNs, WINDOW, T0).ns, T0);
}

TEST(OdometryClock, withoutAWindowInstantTheRosClockIsTheWindowsEnd) {
	OdometryClock clock;
	EXPECT_EQ(clock.next(TwistSource::EncoderWindow, kNoInstantNs, WINDOW, T0).ns, T0 - 25 * MS);
	EXPECT_EQ(clock.next(TwistSource::Held, kNoInstantNs, WINDOW, T0 + 50 * MS).ns, T0);
}

TEST(OdometryClock, aWindowWithoutALengthIsStampedAtItsEnd) {
	OdometryClock clock;
	EXPECT_EQ(clock.next(TwistSource::EncoderWindow, T0, 0.0, T0).ns, T0);
	EXPECT_EQ(clock.next(TwistSource::EncoderWindow, T0, -0.05, T0).ns, T0);
	EXPECT_EQ(clock.next(TwistSource::EncoderWindow, T0, std::numeric_limits<double>::quiet_NaN(), T0).ns, T0);
}

TEST(OdometryClock, stampsNeverRunBackwardsThroughAHoldAndAResync) {
	// Windows run from the last used sample to the newest, so each midpoint
	// lies after the instant a held step repeats.
	OdometryClock clock;
	const struct {
		TwistSource source;
		int64_t endNs;
		double window;
	} steps[] = {
		{TwistSource::DriveVelocity, T0, WINDOW},
		{TwistSource::EncoderWindow, T0 + 50 * MS, WINDOW},
		{TwistSource::Held, T0 + 100 * MS, WINDOW},
		{TwistSource::EncoderWindow, T0 + 150 * MS, 0.1},
		{TwistSource::Held, kNoInstantNs, WINDOW},
		{TwistSource::Held, kNoInstantNs, WINDOW},
		{TwistSource::DriveVelocity, T0 + 400 * MS, WINDOW},
		{TwistSource::EncoderWindow, T0 + 450 * MS, WINDOW},
	};
	int64_t previous = std::numeric_limits<int64_t>::min();
	int i = 0;
	for (const auto& step : steps) {
		const int64_t ns = clock.next(step.source, step.endNs, step.window, T0 + 500 * MS).ns;
		EXPECT_GE(ns, previous) << "step " << i;
		previous = ns;
		i++;
	}
}

}  // namespace
