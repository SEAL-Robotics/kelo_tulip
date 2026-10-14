#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <thread>

#include "kelo_tulip/SampleInstant.h"

namespace {

using kelo::kNoInstantNs;
using kelo::SampleInstant;
using kelo::SampleInstantSlot;

const int64_t MS = 1000000;
const int64_t T0 = 1760000000LL * 1000000000LL;  // a CLOCK_REALTIME instant
const int64_t MAX_IN_FLIGHT = 500 * MS;

TEST(FrameInstant, aFrameIsStampedWithTheInstantItWasSent) {
	EXPECT_EQ(kelo::frameInstantNs(T0, T0 + 900000, MAX_IN_FLIGHT), T0);
}

TEST(FrameInstant, aStalledCycleStillReadsTheFrameItSent) {
	EXPECT_EQ(kelo::frameInstantNs(T0, T0 + 40 * MS, MAX_IN_FLIGHT), T0);
}

TEST(FrameInstant, withoutASendInstantTheArrivalIsTheBound) {
	EXPECT_EQ(kelo::frameInstantNs(kNoInstantNs, T0, MAX_IN_FLIGHT), T0);
}

TEST(FrameInstant, aClockStepFallsBackToTheArrival) {
	EXPECT_EQ(kelo::frameInstantNs(T0, T0 - 1, MAX_IN_FLIGHT), T0 - 1);
	EXPECT_EQ(kelo::frameInstantNs(T0, T0 + 2000 * MS, MAX_IN_FLIGHT), T0 + 2000 * MS);
}

TEST(SampleInstantOf, theCapturedSampleHasItsCapturedInstant) {
	const SampleInstant captured{5000000000ULL, T0};
	EXPECT_EQ(kelo::sampleInstantNs(captured, 5000000000ULL, 100 * MS), T0);
}

TEST(SampleInstantOf, aNeighbouringSampleIsPlacedByTheDrivesCounter) {
	const SampleInstant captured{5000000000ULL, T0};
	EXPECT_EQ(kelo::sampleInstantNs(captured, 5001000000ULL, 100 * MS), T0 + MS);
	EXPECT_EQ(kelo::sampleInstantNs(captured, 4999000000ULL, 100 * MS), T0 - MS);
}

TEST(SampleInstantOf, aCounterTooFarFromTheCaptureHasNoInstant) {
	const SampleInstant captured{5000000000ULL, T0};
	EXPECT_EQ(kelo::sampleInstantNs(captured, 5000000000ULL + 100 * MS + 1, 100 * MS), kNoInstantNs);
	EXPECT_EQ(kelo::sampleInstantNs(captured, 5000000000ULL - 100 * MS - 1, 100 * MS), kNoInstantNs);
	// A drive that restarted counts from zero again.
	EXPECT_EQ(kelo::sampleInstantNs(captured, 1000, 100 * MS), kNoInstantNs);
}

TEST(SampleInstantOf, nothingCapturedIsNoInstant) {
	EXPECT_EQ(kelo::sampleInstantNs(SampleInstant{}, 5000000000ULL, 100 * MS), kNoInstantNs);
}

TEST(SampleInstantSlot, emptyUntilTheFirstPublish) {
	SampleInstantSlot slot;
	SampleInstant out;
	EXPECT_FALSE(slot.read(out));
}

TEST(SampleInstantSlot, readsTheLatestPublishedPair) {
	SampleInstantSlot slot;
	slot.publish(SampleInstant{1, T0});
	slot.publish(SampleInstant{2, T0 + MS});
	SampleInstant out;
	ASSERT_TRUE(slot.read(out));
	EXPECT_EQ(out.sensorTs, 2u);
	EXPECT_EQ(out.hostNs, T0 + MS);
}

TEST(SampleInstantSlot, aReadDuringPublishingNeverPairsOneSampleWithAnothersInstant) {
	SampleInstantSlot slot;
	std::atomic<bool> stop{false};
	std::thread writer([&] {
		for (uint64_t ts = 1; !stop.load(); ts++)
			slot.publish(SampleInstant{ts, static_cast<int64_t>(3 * ts + 7)});
	});
	SampleInstant out;
	while (!slot.read(out))
		std::this_thread::yield();

	int whole = 0;
	int torn = 0;
	for (int i = 0; i < 200000; i++) {
		if (!slot.read(out))
			continue;
		if (out.hostNs != static_cast<int64_t>(3 * out.sensorTs + 7))
			torn++;
		whole++;
	}
	stop = true;
	writer.join();
	EXPECT_EQ(torn, 0) << "torn pairs";
	EXPECT_GT(whole, 0);
}

}  // namespace
