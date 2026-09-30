#include <gtest/gtest.h>

#include "kelo_tulip/CurrentShaping.h"

using kelo::CurrentShapingConfig;

TEST(SlewLimit, DisabledPassesTargetThrough) {
	EXPECT_FLOAT_EQ(kelo::slewLimit(0.0f, 50.0f, 0.0f, 0.001f), 50.0f);
}

TEST(SlewLimit, GrowthIsRateLimited) {
	EXPECT_FLOAT_EQ(kelo::slewLimit(0.0f, 50.0f, 100.0f, 0.1f), 10.0f);
	EXPECT_FLOAT_EQ(kelo::slewLimit(-5.0f, -50.0f, 100.0f, 0.1f), -15.0f);
}

TEST(SlewLimit, ReachesTargetWithoutOvershoot) {
	EXPECT_FLOAT_EQ(kelo::slewLimit(48.0f, 50.0f, 100.0f, 0.1f), 50.0f);
}

TEST(SlewLimit, ShrinkingMagnitudeIsNotDelayed) {
	EXPECT_FLOAT_EQ(kelo::slewLimit(50.0f, 10.0f, 100.0f, 0.001f), 10.0f);
	EXPECT_FLOAT_EQ(kelo::slewLimit(50.0f, 0.0f, 100.0f, 0.001f), 0.0f);
}

TEST(SlewLimit, SignFlipPassesThroughZeroThenRamps) {
	EXPECT_FLOAT_EQ(kelo::slewLimit(50.0f, -40.0f, 100.0f, 0.1f), -10.0f);
}

TEST(SlewLimit, OutputNeverExceedsTargetMagnitude) {
	for (float prev = -60.0f; prev <= 60.0f; prev += 7.0f) {
		for (float target = -60.0f; target <= 60.0f; target += 5.0f) {
			EXPECT_LE(std::fabs(kelo::slewLimit(prev, target, 30.0f, 0.02f)), std::fabs(target));
		}
	}
}

TEST(SlewLimit, ZeroDtHoldsGrowth) {
	EXPECT_FLOAT_EQ(kelo::slewLimit(5.0f, 50.0f, 100.0f, 0.0f), 5.0f);
}

TEST(PivotCorrection, DefaultsMatchLegacyGainAndClip) {
	CurrentShapingConfig cfg;
	EXPECT_NEAR(kelo::pivotCorrectionSpeed(0.1f, cfg), 0.02f, 1e-6f);
	EXPECT_NEAR(kelo::pivotCorrectionSpeed(3.0f, cfg), 0.2f * static_cast<float>(M_PI) * 0.25f, 1e-6f);
	EXPECT_NEAR(kelo::pivotCorrectionSpeed(-3.0f, cfg), -0.2f * static_cast<float>(M_PI) * 0.25f, 1e-6f);
}

TEST(PivotCorrection, CustomGainScalesCorrection) {
	CurrentShapingConfig cfg;
	cfg.pivotKp = 0.1f;
	EXPECT_NEAR(kelo::pivotCorrectionSpeed(0.5f, cfg), 0.05f, 1e-6f);
}

TEST(PivotCorrection, SpeedCapIsSymmetric) {
	CurrentShapingConfig cfg;
	cfg.maxPivotCorrectionSpeed = 0.05f;
	EXPECT_FLOAT_EQ(kelo::pivotCorrectionSpeed(3.0f, cfg), 0.05f);
	EXPECT_FLOAT_EQ(kelo::pivotCorrectionSpeed(-3.0f, cfg), -0.05f);
	EXPECT_NEAR(kelo::pivotCorrectionSpeed(0.1f, cfg), 0.02f, 1e-6f);
}

TEST(ReorientScale, DisabledByDefault) {
	CurrentShapingConfig cfg;
	EXPECT_FLOAT_EQ(kelo::reorientScale(3.0f, cfg), 1.0f);
}

TEST(ReorientScale, FullSpeedNearTargetMinimumFarAway) {
	CurrentShapingConfig cfg;
	cfg.reorientStartError = 0.3f;
	cfg.reorientFullError = 1.3f;
	cfg.reorientMinScale = 0.2f;
	EXPECT_FLOAT_EQ(kelo::reorientScale(0.2f, cfg), 1.0f);
	EXPECT_FLOAT_EQ(kelo::reorientScale(1.3f, cfg), 0.2f);
	EXPECT_FLOAT_EQ(kelo::reorientScale(-3.0f, cfg), 0.2f);
	EXPECT_NEAR(kelo::reorientScale(0.8f, cfg), 0.6f, 1e-6f);
}

TEST(ReorientScale, NeverAboveOne) {
	CurrentShapingConfig cfg;
	cfg.reorientStartError = 0.3f;
	cfg.reorientMinScale = 0.5f;
	for (float e = 0.0f; e < 4.0f; e += 0.1f) {
		EXPECT_LE(kelo::reorientScale(e, cfg), 1.0f);
		EXPECT_GE(kelo::reorientScale(e, cfg), 0.5f);
	}
}

TEST(HubSetpoint, NeverLargerThanLegacyWhenCorrectionIsCapped) {
	// legacy: +0.15 - 0.157 = -0.007; a capped correction must not turn that
	// into +0.10
	const float legacyDelta = -0.157f;
	const float out = kelo::hubSetpoint(0.15f, -0.05f, legacyDelta, 1.0f);
	EXPECT_LE(std::fabs(out), std::fabs(0.15f + legacyDelta) + 1e-6f);
}

TEST(HubSetpoint, FallsBackToLegacyWhenShapedIsLarger) {
	EXPECT_NEAR(kelo::hubSetpoint(0.15f, -0.05f, -0.157f, 1.0f), -0.007f, 1e-6f);
}

TEST(HubSetpoint, OppositeSignToLegacyBecomesZero) {
	// shaped 0.05 - 0.06 = -0.01 is smaller than legacy 0.10 but points the
	// other way
	EXPECT_FLOAT_EQ(kelo::hubSetpoint(0.05f, -0.06f, 0.05f, 1.0f), 0.0f);
}

TEST(HubSetpoint, ScalingPicksSmallerMagnitude) {
	EXPECT_NEAR(kelo::hubSetpoint(0.5f, 0.02f, 0.02f, 0.25f), 0.145f, 1e-6f);
}

TEST(IsValid, DefaultsAreValid) {
	EXPECT_TRUE(kelo::isValid(CurrentShapingConfig()));
}

TEST(IsValid, RejectsNonFinite) {
	const float nan = std::nanf("");
	const float inf = INFINITY;
	CurrentShapingConfig c;
	c.slewRateRadPerSecSq = nan;
	EXPECT_FALSE(kelo::isValid(c));
	c = CurrentShapingConfig();
	c.maxPivotError = inf;
	EXPECT_FALSE(kelo::isValid(c));
	c = CurrentShapingConfig();
	c.reorientFullError = nan;
	EXPECT_FALSE(kelo::isValid(c));
	c = CurrentShapingConfig();
	c.maxPivotCorrectionSpeed = -inf;
	EXPECT_FALSE(kelo::isValid(c));
}

TEST(IsValid, RejectsOutOfRange) {
	std::string why;
	CurrentShapingConfig c;
	c.pivotKp = 0.3f;
	EXPECT_FALSE(kelo::isValid(c, &why));
	EXPECT_FALSE(why.empty());
	c = CurrentShapingConfig();
	c.pivotKp = -0.1f;
	EXPECT_FALSE(kelo::isValid(c));
	c = CurrentShapingConfig();
	c.maxPivotError = 1.0f;
	EXPECT_FALSE(kelo::isValid(c));
	c.maxPivotError = 0.0f;
	EXPECT_FALSE(kelo::isValid(c));
	c = CurrentShapingConfig();
	c.slewRateRadPerSecSq = -1.0f;
	EXPECT_FALSE(kelo::isValid(c));
	c = CurrentShapingConfig();
	c.maxPivotCorrectionSpeed = -0.1f;
	EXPECT_FALSE(kelo::isValid(c));
	c = CurrentShapingConfig();
	c.reorientMinScale = 1.5f;
	EXPECT_FALSE(kelo::isValid(c));
}

TEST(IsValid, ReorientNeedsOrderedThresholdsAndFloorScale) {
	CurrentShapingConfig c;
	c.reorientStartError = 0.5f;
	c.reorientFullError = 0.5f;
	c.reorientMinScale = 0.5f;
	EXPECT_FALSE(kelo::isValid(c));
	c.reorientFullError = 1.0f;
	EXPECT_TRUE(kelo::isValid(c));
	c.reorientMinScale = 0.05f;
	EXPECT_FALSE(kelo::isValid(c));
	c.reorientMinScale = 0.1f;
	EXPECT_TRUE(kelo::isValid(c));
}
