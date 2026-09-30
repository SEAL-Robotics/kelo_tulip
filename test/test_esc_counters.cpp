#include <gtest/gtest.h>

#include <cstring>

#include "kelo_tulip/EscCounters.h"

using kelo::decodeEscCounters;
using kelo::describeEscDelta;
using kelo::EscCounters;
using kelo::EscCounterTracker;

namespace {

TEST(DecodeEscCounters, readsEveryRegisterFromItsOffset) {
	std::uint8_t raw[kelo::kEscErrorRegLength];
	std::memset(raw, 0, sizeof(raw));
	// 0x0300/0x0301 are port 0's invalid-frame and RX-error counters, 0x0302/3 port 1's.
	raw[0x00] = 1;
	raw[0x01] = 2;
	raw[0x02] = 3;
	raw[0x03] = 4;
	raw[0x06] = 5;
	raw[0x07] = 6;
	raw[0x08] = 7;   // forwarded RX error, port 0
	raw[0x0B] = 8;   // forwarded RX error, port 3
	raw[0x0C] = 9;   // ECAT processing unit error
	raw[0x0D] = 10;  // PDI error
	raw[0x10] = 11;  // lost link, port 0
	raw[0x13] = 12;  // lost link, port 3

	const EscCounters c = decodeEscCounters(raw);

	EXPECT_EQ(c.invalidFrame[0], 1);
	EXPECT_EQ(c.rxError[0], 2);
	EXPECT_EQ(c.invalidFrame[1], 3);
	EXPECT_EQ(c.rxError[1], 4);
	EXPECT_EQ(c.invalidFrame[3], 5);
	EXPECT_EQ(c.rxError[3], 6);
	EXPECT_EQ(c.forwardedRxError[0], 7);
	EXPECT_EQ(c.forwardedRxError[3], 8);
	EXPECT_EQ(c.processingUnitError, 9);
	EXPECT_EQ(c.pdiError, 10);
	EXPECT_EQ(c.lostLink[0], 11);
	EXPECT_EQ(c.lostLink[3], 12);
}

TEST(EscCounterTracker, firstReadIsABaselineNotAnError) {
	EscCounterTracker tracker;
	EscCounters c;
	c.rxError[0] = 40;

	const auto delta = tracker.update(1, c);

	EXPECT_TRUE(delta.firstRead);
	EXPECT_FALSE(delta.any());
}

TEST(EscCounterTracker, reportsWhatGrewSinceThePreviousRead) {
	EscCounterTracker tracker;
	EscCounters c;
	c.rxError[0] = 10;
	tracker.update(1, c);
	c.rxError[0] = 13;
	c.forwardedRxError[1] = 2;
	c.lostLink[0] = 1;

	const auto delta = tracker.update(1, c);

	EXPECT_FALSE(delta.firstRead);
	EXPECT_TRUE(delta.any());
	EXPECT_EQ(delta.rxError[0], 3u);
	EXPECT_EQ(delta.forwardedRxError[1], 2u);
	EXPECT_EQ(delta.lostLink[0], 1u);
	EXPECT_EQ(delta.rxError[1], 0u);
}

TEST(EscCounterTracker, unchangedCountersGiveNoDelta) {
	EscCounterTracker tracker;
	EscCounters c;
	c.rxError[2] = 7;
	tracker.update(1, c);

	EXPECT_FALSE(tracker.update(1, c).any());
}

TEST(EscCounterTracker, slavesAreTrackedIndependently) {
	EscCounterTracker tracker;
	EscCounters a;
	EscCounters b;
	a.rxError[0] = 5;
	tracker.update(1, a);
	tracker.update(4, b);
	b.rxError[0] = 1;

	const auto delta = tracker.update(4, b);

	EXPECT_EQ(delta.rxError[0], 1u);
	EXPECT_FALSE(tracker.update(1, a).any());
}

TEST(EscCounterTracker, aCounterGoingBackwardsMeansTheSlaveRestarted) {
	EscCounterTracker tracker;
	EscCounters c;
	c.rxError[0] = 100;
	tracker.update(1, c);
	c.rxError[0] = 3;

	const auto delta = tracker.update(1, c);

	EXPECT_TRUE(delta.restarted);
	EXPECT_EQ(delta.rxError[0], 3u);
}

TEST(EscCounterTracker, flagsSaturatedCounters) {
	EscCounterTracker tracker;
	EscCounters c;
	c.rxError[0] = 255;
	tracker.update(1, c);

	EXPECT_TRUE(tracker.update(1, c).saturated);
}

TEST(DescribeEscDelta, namesEachChangedCounterAndItsPort) {
	EscCounterTracker tracker;
	EscCounters c;
	tracker.update(2, c);
	c.rxError[1] = 4;
	c.lostLink[0] = 1;

	const std::string text = describeEscDelta(tracker.update(2, c));

	EXPECT_NE(text.find("rx_error[p1] +4"), std::string::npos) << text;
	EXPECT_NE(text.find("lost_link[p0] +1"), std::string::npos) << text;
	EXPECT_EQ(text.find("invalid_frame"), std::string::npos) << text;
}

}  // namespace
