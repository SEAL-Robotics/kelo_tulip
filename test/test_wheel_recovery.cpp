#include <gtest/gtest.h>

#include "kelo_tulip/WheelRecovery.h"

using kelo::effectiveWheelEnable;
using kelo::InitAction;
using kelo::initTimeoutAction;
using kelo::kStatus1Healthy;
using kelo::kStatus2Healthy;
using kelo::wheelNeedsRecovery;

namespace {

TEST(WheelNeedsRecovery, healthyEnabledWheelIsLeftAlone) {
	EXPECT_FALSE(wheelNeedsRecovery(kStatus1Healthy, kStatus2Healthy, true));
}

TEST(WheelNeedsRecovery, faultedEnabledWheelIsRecovered) {
	EXPECT_TRUE(wheelNeedsRecovery(61, kStatus2Healthy, true));
	EXPECT_TRUE(wheelNeedsRecovery(kStatus1Healthy, 18435, true));
}

TEST(WheelNeedsRecovery, disabledWheelIsNeverRecovered) {
	// A deliberately disabled wheel reports status1 without the enable bits;
	// recovering it would re-enable a wheel someone switched off.
	EXPECT_FALSE(wheelNeedsRecovery(60, kStatus2Healthy, false));
	EXPECT_FALSE(wheelNeedsRecovery(61, 18435, false));
}

TEST(EffectiveWheelEnable, operatorDisableAlwaysWins) {
	EXPECT_FALSE(effectiveWheelEnable(false, true));
	EXPECT_FALSE(effectiveWheelEnable(false, false));
}

TEST(EffectiveWheelEnable, recoveryCanOnlyWithholdTheEnable) {
	EXPECT_TRUE(effectiveWheelEnable(true, true));
	EXPECT_FALSE(effectiveWheelEnable(true, false));
}

TEST(InitTimeoutAction, waitsWhileReadyOrWithinTheTimeout) {
	EXPECT_EQ(initTimeoutAction(true, 10000, 500, 0, 3), InitAction::Wait);
	EXPECT_EQ(initTimeoutAction(false, 500, 500, 0, 3), InitAction::Wait);
}

TEST(InitTimeoutAction, resetsTheWheelsAfterATimeout) {
	EXPECT_EQ(initTimeoutAction(false, 501, 500, 0, 3), InitAction::ResetWheels);
	EXPECT_EQ(initTimeoutAction(false, 501, 500, 2, 3), InitAction::ResetWheels);
}

TEST(InitTimeoutAction, givesUpOnceTheResetsAreSpent) {
	EXPECT_EQ(initTimeoutAction(false, 501, 500, 3, 3), InitAction::GiveUp);
	EXPECT_EQ(initTimeoutAction(false, 501, 500, 0, 0), InitAction::GiveUp);
}

}  // namespace
