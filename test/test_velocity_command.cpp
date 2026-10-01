#include <gtest/gtest.h>

#include <atomic>
#include <limits>
#include <thread>

#include "kelo_tulip/VelocityCommand.h"

using kelo::CommandMailbox;
using kelo::StampedCommand;
using kelo::VelocityCommand;

TEST(SanitizeCommand, aFiniteCommandIsKept) {
	VelocityCommand c{0.5, -0.2, 0.1};
	EXPECT_TRUE(kelo::sanitizeCommand(c));
	EXPECT_DOUBLE_EQ(c.vx, 0.5);
	EXPECT_DOUBLE_EQ(c.vy, -0.2);
	EXPECT_DOUBLE_EQ(c.va, 0.1);
}

TEST(SanitizeCommand, anyNonFiniteAxisMakesTheWholeCommandAStop) {
	const double bad[] = {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
		-std::numeric_limits<double>::infinity()};
	for (const double b : bad) {
		for (int axis = 0; axis < 3; axis++) {
			VelocityCommand c{0.5, 0.5, 0.5};
			(axis == 0 ? c.vx : axis == 1 ? c.vy : c.va) = b;
			EXPECT_FALSE(kelo::sanitizeCommand(c));
			EXPECT_EQ(c.vx, 0.0);
			EXPECT_EQ(c.vy, 0.0);
			EXPECT_EQ(c.va, 0.0);
		}
	}
}

TEST(SanitizeCommand, aValueBeyondTheFloatRangeIsAStop) {
	VelocityCommand c{1.0e300, 0.0, 0.0};
	EXPECT_FALSE(kelo::sanitizeCommand(c));
	EXPECT_EQ(c.vx, 0.0);
}

TEST(CommandExpired, nothingCommandedIsNeverExpired) {
	EXPECT_FALSE(kelo::commandExpired(StampedCommand(), 1.0e9, 200.0));
}

TEST(CommandExpired, expiresOnlyPastTheTimeout) {
	StampedCommand c;
	c.seq = 1;
	c.stampMs = 1000.0;
	EXPECT_FALSE(kelo::commandExpired(c, 1000.0, 200.0));
	EXPECT_FALSE(kelo::commandExpired(c, 1200.0, 200.0));
	EXPECT_TRUE(kelo::commandExpired(c, 1200.5, 200.0));
}

TEST(CommandExpired, aCommandStampedJustAfterTheCycleReadTheClockIsFresh) {
	StampedCommand c;
	c.seq = 1;
	c.stampMs = 1000.2;
	EXPECT_FALSE(kelo::commandExpired(c, 1000.0, 200.0));
}

TEST(CommandExpired, aNonFiniteClockOrStampExpires) {
	StampedCommand c;
	c.seq = 1;
	c.stampMs = 1000.0;
	EXPECT_TRUE(kelo::commandExpired(c, std::numeric_limits<double>::quiet_NaN(), 200.0));
	c.stampMs = std::numeric_limits<double>::quiet_NaN();
	EXPECT_TRUE(kelo::commandExpired(c, 1000.0, 200.0));
}

TEST(CommandMailbox, startsEmptyAndHandsOverTheLatest) {
	CommandMailbox box;
	StampedCommand out;
	ASSERT_TRUE(box.tryFetch(out));
	EXPECT_EQ(out.seq, 0u);

	box.post(VelocityCommand{0.1, 0.0, 0.0}, 5.0);
	box.post(VelocityCommand{0.3, 0.0, 0.0}, 6.0);
	ASSERT_TRUE(box.tryFetch(out));
	EXPECT_EQ(out.seq, 2u);
	EXPECT_DOUBLE_EQ(out.command.vx, 0.3);
	EXPECT_DOUBLE_EQ(out.stampMs, 6.0);
}

// A command is read whole: never one axis from one post and one from another.
TEST(CommandMailbox, aReaderNeverSeesATornCommand) {
	CommandMailbox box;
	std::atomic<bool> done{false};
	std::thread writer([&] {
		for (int i = 1; i <= 200000; i++)
			box.post(VelocityCommand{double(i), double(i), double(i)}, double(i));
		done = true;
	});
	StampedCommand out;
	while (!done.load()) {
		if (box.tryFetch(out) && out.seq > 0) {
			ASSERT_EQ(out.command.vx, out.command.vy);
			ASSERT_EQ(out.command.vx, out.command.va);
			ASSERT_EQ(out.command.vx, out.stampMs);
		}
	}
	writer.join();
}
