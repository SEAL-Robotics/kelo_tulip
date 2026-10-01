#include <gtest/gtest.h>

#include <limits>

#include "kelo_tulip/RampTiming.h"

using kelo::rampStep;

TEST(RampStep, aNormalCyclePassesThrough) {
	const kelo::RampStep step = rampStep(0.001f);
	EXPECT_FLOAT_EQ(step.accelDt, 0.001f);
	EXPECT_FLOAT_EQ(step.decelDt, 0.001f);
	EXPECT_FALSE(step.paused);
}

TEST(RampStep, aBackwardsOrZeroStepRampsByNothing) {
	for (const float dt : {0.0f, -0.001f, -3600.0f}) {
		const kelo::RampStep step = rampStep(dt);
		EXPECT_EQ(step.accelDt, 0.0f) << dt;
		EXPECT_EQ(step.decelDt, 0.0f) << dt;
		EXPECT_FALSE(step.paused) << dt;
	}
}

TEST(RampStep, aNonFiniteStepRampsByNothing) {
	for (const float dt : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(),
			-std::numeric_limits<float>::infinity()}) {
		const kelo::RampStep step = rampStep(dt);
		EXPECT_EQ(step.accelDt, 0.0f);
		EXPECT_EQ(step.decelDt, 0.0f);
	}
}

TEST(RampStep, accelerationNeverSeesMoreThanTheClamp) {
	EXPECT_FLOAT_EQ(rampStep(0.05f).accelDt, kelo::MAX_ACCEL_DT_SEC);
	EXPECT_FLOAT_EQ(rampStep(3600.0f).accelDt, kelo::MAX_ACCEL_DT_SEC);
}

TEST(RampStep, brakingKeepsUpWithASlowLoopUpToThePauseGap) {
	EXPECT_FLOAT_EQ(rampStep(0.05f).decelDt, 0.05f);
	EXPECT_FLOAT_EQ(rampStep(3600.0f).decelDt, kelo::MAX_DECEL_DT_SEC);
}

TEST(RampStep, aGapAboveThePauseGapIsAPause) {
	EXPECT_FALSE(rampStep(kelo::PAUSE_GAP_SEC).paused);
	EXPECT_TRUE(rampStep(kelo::PAUSE_GAP_SEC * 1.5f).paused);
}
