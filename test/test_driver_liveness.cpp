#include <gtest/gtest.h>

#include "kelo_tulip/DriverLiveness.h"

using kelo::LoopVerdict;
using kelo::checkEthercatLiveness;

namespace {

TEST(DriverLiveness, keepsRunningWhileEthercatRuns) {
	EXPECT_EQ(checkEthercatLiveness(false, false, false), LoopVerdict::Continue);
	EXPECT_EQ(checkEthercatLiveness(true, true, false), LoopVerdict::Continue);
}

TEST(DriverLiveness, exitsOnceTheEthercatLoopHasStopped) {
	EXPECT_EQ(checkEthercatLiveness(false, false, true), LoopVerdict::EthercatStopped);
}

TEST(DriverLiveness, exitsWhenReinitializationFails) {
	EXPECT_EQ(checkEthercatLiveness(true, false, false), LoopVerdict::ReinitFailed);
	EXPECT_EQ(checkEthercatLiveness(true, false, true), LoopVerdict::ReinitFailed);
}

TEST(DriverLiveness, onlyContinueMapsToExitCodeZero) {
	EXPECT_EQ(exitCode(LoopVerdict::Continue), 0);
	EXPECT_NE(exitCode(LoopVerdict::EthercatStopped), 0);
	EXPECT_NE(exitCode(LoopVerdict::ReinitFailed), 0);
}

}  // namespace
