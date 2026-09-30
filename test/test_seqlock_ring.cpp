#include <gtest/gtest.h>

#include <atomic>
#include <thread>

#include "kelo_tulip/SeqlockRing.h"

using kelo::SeqlockRing;

namespace {

struct Sample {
	std::uint64_t index;
	std::uint64_t mirror;  // always index * 3, so a torn read shows
	std::uint64_t pad[2];
};

Sample makeSample(std::uint64_t i) { return Sample{i, i * 3, {i, i}}; }

TEST(SeqlockRing, startsEmpty) {
	SeqlockRing<Sample> ring(8);
	std::vector<Sample> out;
	EXPECT_EQ(ring.writeCount(), 0u);
	EXPECT_EQ(ring.snapshot(out, 8), 0u);
	EXPECT_TRUE(out.empty());
}

TEST(SeqlockRing, snapshotReturnsSamplesOldestFirst) {
	SeqlockRing<Sample> ring(8);
	for (std::uint64_t i = 0; i < 5; i++)
		ring.push(makeSample(i));

	std::vector<Sample> out;
	ASSERT_EQ(ring.snapshot(out, 8), 5u);
	for (std::uint64_t i = 0; i < 5; i++)
		EXPECT_EQ(out[i].index, i);
}

TEST(SeqlockRing, keepsOnlyTheNewestCapacitySamplesOnceFull) {
	SeqlockRing<Sample> ring(8);
	for (std::uint64_t i = 0; i < 20; i++)
		ring.push(makeSample(i));

	std::vector<Sample> out;
	ASSERT_EQ(ring.snapshot(out, 8), 8u);
	EXPECT_EQ(out.front().index, 12u);
	EXPECT_EQ(out.back().index, 19u);
	EXPECT_EQ(ring.writeCount(), 20u);
}

TEST(SeqlockRing, snapshotHonoursTheRequestedCount) {
	SeqlockRing<Sample> ring(8);
	for (std::uint64_t i = 0; i < 8; i++)
		ring.push(makeSample(i));

	std::vector<Sample> out;
	ASSERT_EQ(ring.snapshot(out, 3), 3u);
	EXPECT_EQ(out.front().index, 5u);
	EXPECT_EQ(out.back().index, 7u);
}

TEST(SeqlockRing, aSnapshotTakenWhileTheProducerRunsNeverContainsATornSample) {
	SeqlockRing<Sample> ring(64);
	std::atomic<bool> stop{false};
	std::thread producer([&] {
		for (std::uint64_t i = 0; !stop.load(); i++)
			ring.push(makeSample(i));
	});
	while (ring.writeCount() == 0)
		std::this_thread::yield();

	// Whatever the scheduler does, a sample that comes back is whole and in
	// order; how many come back depends on timing and is not asserted.
	std::vector<Sample> out;
	std::size_t returned = 0;
	for (int round = 0; round < 2000; round++) {
		returned += ring.snapshot(out, 64);
		std::uint64_t previous = 0;
		bool first = true;
		for (const Sample& s : out) {
			ASSERT_EQ(s.mirror, s.index * 3) << "torn sample";
			ASSERT_EQ(s.pad[0], s.index);
			ASSERT_EQ(s.pad[1], s.index);
			if (!first) {
				ASSERT_GT(s.index, previous) << "out of order";
			}
			previous = s.index;
			first = false;
		}
	}
	stop = true;
	producer.join();
	EXPECT_GT(returned, 0u) << "no snapshot ever returned a sample";

	// With the producer quiet the snapshot is complete, deterministically.
	ASSERT_EQ(ring.snapshot(out, 64), 64u);
	for (std::size_t i = 1; i < out.size(); i++)
		ASSERT_EQ(out[i].index, out[i - 1].index + 1);
}

}  // namespace
