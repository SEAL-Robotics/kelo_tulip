#include <gtest/gtest.h>

#include <limits>

#include "kelo_tulip/CommandWatchdog.h"

using kelo::CommandWatchdog;
using std::chrono::milliseconds;

namespace {

const CommandWatchdog::Clock::time_point t0 = CommandWatchdog::Clock::time_point() + std::chrono::hours(1);

TEST(CommandWatchdog, doesNotExpireBeforeFirstCommand) {
	CommandWatchdog watchdog(0.2);

	EXPECT_FALSE(watchdog.checkExpired(t0));
	EXPECT_FALSE(watchdog.checkExpired(t0 + milliseconds(10000)));
}

TEST(CommandWatchdog, doesNotExpireWithinTimeout) {
	CommandWatchdog watchdog(0.2);
	watchdog.kick(t0);

	EXPECT_FALSE(watchdog.checkExpired(t0 + milliseconds(100)));
	EXPECT_FALSE(watchdog.checkExpired(t0 + milliseconds(200)));
}

TEST(CommandWatchdog, expiresOnceAfterTimeout) {
	CommandWatchdog watchdog(0.2);
	watchdog.kick(t0);

	EXPECT_TRUE(watchdog.checkExpired(t0 + milliseconds(201)));
	EXPECT_FALSE(watchdog.checkExpired(t0 + milliseconds(250)));
	EXPECT_FALSE(watchdog.checkExpired(t0 + milliseconds(5000)));
}

TEST(CommandWatchdog, newCommandRearms) {
	CommandWatchdog watchdog(0.2);
	watchdog.kick(t0);
	ASSERT_TRUE(watchdog.checkExpired(t0 + milliseconds(300)));

	watchdog.kick(t0 + milliseconds(400));

	EXPECT_FALSE(watchdog.checkExpired(t0 + milliseconds(550)));
	EXPECT_TRUE(watchdog.checkExpired(t0 + milliseconds(650)));
}

TEST(CommandWatchdog, steadyStreamNeverExpires) {
	CommandWatchdog watchdog(0.2);

	// 20 Hz commands, checked at 20 Hz in between, for 10 s.
	for (int ms = 0; ms < 10000; ms += 50) {
		watchdog.kick(t0 + milliseconds(ms));
		EXPECT_FALSE(watchdog.checkExpired(t0 + milliseconds(ms + 25)));
	}
}

TEST(CommandWatchdog, timeoutIsConfigurable) {
	CommandWatchdog watchdog;
	EXPECT_DOUBLE_EQ(watchdog.getTimeout(), 0.2);

	watchdog.setTimeout(0.5);
	EXPECT_DOUBLE_EQ(watchdog.getTimeout(), 0.5);

	watchdog.kick(t0);
	EXPECT_FALSE(watchdog.checkExpired(t0 + milliseconds(400)));
	EXPECT_TRUE(watchdog.checkExpired(t0 + milliseconds(501)));
}

TEST(CommandWatchdog, rejectsTimeoutsOutsideValidRange) {
	CommandWatchdog watchdog(0.2);
	const double invalid[] = {0.0, -0.1, 2.001, 200.0,
		std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()};

	for (double timeout : invalid) {
		EXPECT_FALSE(watchdog.setTimeout(timeout)) << timeout;
		EXPECT_DOUBLE_EQ(watchdog.getTimeout(), 0.2) << timeout;
	}
}

TEST(CommandWatchdog, acceptsMaximumTimeout) {
	CommandWatchdog watchdog(0.2);

	EXPECT_TRUE(watchdog.setTimeout(CommandWatchdog::MAX_TIMEOUT_SEC));
	EXPECT_DOUBLE_EQ(watchdog.getTimeout(), CommandWatchdog::MAX_TIMEOUT_SEC);
}

} // namespace
