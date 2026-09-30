#include <gtest/gtest.h>

#include "kelo_tulip/DumpPolicy.h"

using kelo::DumpFile;
using kelo::DumpReason;
using kelo::dumpFilesToDelete;
using kelo::TriggerRateLimiter;

namespace {

constexpr std::int64_t kSecond = 1000000000LL;

TEST(TriggerRateLimiter, acceptsTheFirstTrigger) {
	TriggerRateLimiter limiter(10 * kSecond);
	EXPECT_TRUE(limiter.tryAccept(5 * kSecond));
}

TEST(TriggerRateLimiter, dropsTriggersInsideTheIntervalKeepingTheFirst) {
	TriggerRateLimiter limiter(10 * kSecond);
	ASSERT_TRUE(limiter.tryAccept(100 * kSecond));
	EXPECT_FALSE(limiter.tryAccept(101 * kSecond));
	EXPECT_FALSE(limiter.tryAccept(109 * kSecond));
}

TEST(TriggerRateLimiter, acceptsAgainOnceTheIntervalPassed) {
	TriggerRateLimiter limiter(10 * kSecond);
	ASSERT_TRUE(limiter.tryAccept(100 * kSecond));
	EXPECT_TRUE(limiter.tryAccept(110 * kSecond));
	// The dropped triggers must not have pushed the window forward.
	EXPECT_FALSE(limiter.tryAccept(111 * kSecond));
}

TEST(TriggerRateLimiter, aDroppedTriggerDoesNotExtendTheWindow) {
	TriggerRateLimiter limiter(10 * kSecond);
	ASSERT_TRUE(limiter.tryAccept(0));
	EXPECT_FALSE(limiter.tryAccept(9 * kSecond));
	EXPECT_TRUE(limiter.tryAccept(10 * kSecond));
}

TEST(DumpReasons, onlyAWheelGivingUpBypassesTheRateLimit) {
	EXPECT_TRUE(kelo::isCriticalDumpReason(DumpReason::WheelFailed));
	EXPECT_FALSE(kelo::isCriticalDumpReason(DumpReason::WkcError));
	EXPECT_FALSE(kelo::isCriticalDumpReason(DumpReason::SlaveLost));
	EXPECT_FALSE(kelo::isCriticalDumpReason(DumpReason::WheelRecovery));
}

TEST(DumpReasons, haveDistinctFilenameSafeNames) {
	EXPECT_STREQ(kelo::dumpReasonName(DumpReason::WkcError), "wkc_error");
	EXPECT_STREQ(kelo::dumpReasonName(DumpReason::SlaveLost), "slave_lost");
	EXPECT_STREQ(kelo::dumpReasonName(DumpReason::WheelRecovery), "wheel_recovery");
	EXPECT_STREQ(kelo::dumpReasonName(DumpReason::WheelFailed), "wheel_failed");
}

TEST(DumpFilesToDelete, keepsEverythingWhileUnderBothLimits) {
	std::vector<DumpFile> files{{"a", 100}, {"b", 100}};
	EXPECT_TRUE(dumpFilesToDelete(files, 10, 1000, 100).empty());
}

TEST(DumpFilesToDelete, makesRoomForTheIncomingFileByCount) {
	std::vector<DumpFile> files{{"c", 10}, {"a", 10}, {"b", 10}};
	const auto doomed = dumpFilesToDelete(files, 3, 1000, 10);
	// Three files exist and one is coming: the oldest name goes.
	ASSERT_EQ(doomed.size(), 1u);
	EXPECT_EQ(doomed[0], "a");
}

TEST(DumpFilesToDelete, makesRoomByTotalBytesOldestFirst) {
	std::vector<DumpFile> files{{"a", 400}, {"b", 400}, {"c", 400}};
	const auto doomed = dumpFilesToDelete(files, 100, 1000, 400);
	// 1200 + 400 > 1000: drop a (800 + 400 still too big), then b.
	ASSERT_EQ(doomed.size(), 2u);
	EXPECT_EQ(doomed[0], "a");
	EXPECT_EQ(doomed[1], "b");
}

TEST(DumpFilesToDelete, dropsEverythingWhenTheIncomingFileAloneIsOverTheBudget) {
	std::vector<DumpFile> files{{"a", 1}, {"b", 1}};
	EXPECT_EQ(dumpFilesToDelete(files, 100, 500, 900).size(), 2u);
}

TEST(DumpFilesToDelete, ordersByAgeNotByName) {
	// Names from a restarted process sort before older files' names.
	std::vector<DumpFile> files{{"a_new", 10, 300}, {"z_old", 10, 100}, {"m_mid", 10, 200}};
	const auto doomed = dumpFilesToDelete(files, 3, 1000, 10);
	ASSERT_EQ(doomed.size(), 1u);
	EXPECT_EQ(doomed[0], "z_old");
}

TEST(DumpFilesToDelete, aZeroFileBudgetStillLeavesTheNewFile) {
	std::vector<DumpFile> files{{"a", 1}};
	EXPECT_EQ(dumpFilesToDelete(files, 0, 1000, 1).size(), 1u);
}

}  // namespace
