/******************************************************************************
 * This software is published under the same dual-license as the rest of
 * kelo_tulip: GNU Lesser General Public License LGPL 2.1 and BSD license.
 ******************************************************************************/

#ifndef KELOTULIP_SAMPLEINSTANT_H
#define KELOTULIP_SAMPLEINSTANT_H

#include <atomic>
#include <cstdint>
#include <limits>

namespace kelo {

// A drive stamps its samples with its own counter (sensor_ts, ns since its
// power-up, on no clock the host shares), so the host has to say when a
// sample was taken. A frame picks up the drives' inputs as it passes them,
// microseconds after the master sends it, then waits in the socket until the
// next cycle reads it, most of a millisecond later. The send instant is
// therefore the host time of the samples a frame carries, late only by how
// long a drive held its sample before the frame came by.

constexpr int64_t kNoInstantNs = std::numeric_limits<int64_t>::min();

struct SampleInstant {
	uint64_t sensorTs = 0;          // the drive's own counter
	int64_t hostNs = kNoInstantNs;  // host clock, CLOCK_REALTIME
};

// A cycle reads the frame the previous cycle sent. Without that send instant
// (the first cycle after a start reads a frame the start-up sent), or with a
// gap no frame waits (the clock was stepped), the arrival is the bound left.
inline int64_t frameInstantNs(int64_t sentNs, int64_t arrivedNs, int64_t maxInFlightNs) {
	if (sentNs == kNoInstantNs || arrivedNs < sentNs || arrivedNs - sentNs > maxInFlightNs)
		return arrivedNs;
	return sentNs;
}

// Host instant of the sample a wheel reported as sensorTs, from the instant
// captured for one of its samples. The drive's counter bridges a gap of a few
// frames, when the read straddled a cycle; beyond maxGapNs (a drive that
// restarted its counter) the sample has no instant.
inline int64_t sampleInstantNs(const SampleInstant& captured, uint64_t sensorTs, int64_t maxGapNs) {
	if (captured.hostNs == kNoInstantNs || maxGapNs < 0)
		return kNoInstantNs;
	const uint64_t limit = static_cast<uint64_t>(maxGapNs);
	if (sensorTs >= captured.sensorTs) {
		const uint64_t ahead = sensorTs - captured.sensorTs;
		return ahead <= limit ? captured.hostNs + static_cast<int64_t>(ahead) : kNoInstantNs;
	}
	const uint64_t behind = captured.sensorTs - sensorTs;
	return behind <= limit ? captured.hostNs - static_cast<int64_t>(behind) : kNoInstantNs;
}

// A wheel's latest SampleInstant, from the EtherCAT thread to the ROS thread.
// The sequence count around a write lets a reader see that it overlapped one
// and read again, so it never pairs one sample's counter with another's
// instant. publish() never locks, allocates or waits.
class SampleInstantSlot {
	static_assert(std::atomic<uint64_t>::is_always_lock_free, "the realtime thread must never take a lock");
	static_assert(std::atomic<int64_t>::is_always_lock_free, "the realtime thread must never take a lock");

public:
	static constexpr int kReadAttempts = 4;

	// One writer only: the EtherCAT thread.
	void publish(const SampleInstant& sample) {
		const uint64_t seq = sequence_.load(std::memory_order_relaxed);
		sequence_.store(seq + 1, std::memory_order_relaxed);
		std::atomic_thread_fence(std::memory_order_release);
		sensorTs_.store(sample.sensorTs, std::memory_order_relaxed);
		hostNs_.store(sample.hostNs, std::memory_order_relaxed);
		sequence_.store(seq + 2, std::memory_order_release);
	}

	// False before the first publish, or if every attempt overlapped a write.
	bool read(SampleInstant& out) const {
		for (int attempt = 0; attempt < kReadAttempts; attempt++) {
			const uint64_t before = sequence_.load(std::memory_order_acquire);
			if (before == 0)
				return false;
			if (before & 1)
				continue;
			SampleInstant sample;
			sample.sensorTs = sensorTs_.load(std::memory_order_relaxed);
			sample.hostNs = hostNs_.load(std::memory_order_relaxed);
			std::atomic_thread_fence(std::memory_order_acquire);
			if (sequence_.load(std::memory_order_relaxed) == before) {
				out = sample;
				return true;
			}
		}
		return false;
	}

private:
	std::atomic<uint64_t> sequence_{0};
	std::atomic<uint64_t> sensorTs_{0};
	std::atomic<int64_t> hostNs_{kNoInstantNs};
};

}  // namespace kelo

#endif  // KELOTULIP_SAMPLEINSTANT_H
