/******************************************************************************
 * This software is published under the same dual-license as the rest of
 * kelo_tulip: GNU Lesser General Public License LGPL 2.1 and BSD license.
 ******************************************************************************/

#ifndef KELOTULIP_SEQLOCKRING_H
#define KELOTULIP_SEQLOCKRING_H

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <type_traits>
#include <vector>

namespace kelo {

// Single-producer ring of trivially copyable samples that another thread can
// snapshot while the producer keeps writing. push() never allocates, locks or
// waits, so it is safe on the 1 kHz EtherCAT thread. A slot's sequence number
// encodes which push last wrote it; a reader keeps a slot only if that number
// is the one it expects both before and after the copy, so a sample that was
// overwritten mid-read is dropped rather than returned torn. The payload is
// stored in relaxed atomic words to keep the concurrent copy well-defined.
template <typename T>
class SeqlockRing {
	static_assert(std::is_trivially_copyable<T>::value, "samples are copied word by word");
	static_assert(sizeof(T) % sizeof(std::uint64_t) == 0, "sample size must be a multiple of 8 bytes");
	static_assert(std::atomic<std::uint64_t>::is_always_lock_free, "the realtime thread must never take a lock");
	static constexpr std::size_t kWords = sizeof(T) / sizeof(std::uint64_t);

	struct Slot {
		std::atomic<std::uint64_t> sequence{0};
		std::atomic<std::uint64_t> words[kWords];
	};

public:
	explicit SeqlockRing(std::size_t capacity)
		: capacity_(std::max<std::size_t>(capacity, 1))
		, slots_(new Slot[capacity_])
	{
		for (std::size_t i = 0; i < capacity_; i++)
			for (auto& w : slots_[i].words)
				w.store(0, std::memory_order_relaxed);
	}

	std::size_t capacity() const { return capacity_; }
	std::uint64_t writeCount() const { return head_.load(std::memory_order_acquire); }

	void push(const T& sample) {
		const std::uint64_t index = head_.load(std::memory_order_relaxed);
		Slot& slot = slots_[index % capacity_];
		std::uint64_t words[kWords];
		std::memcpy(words, &sample, sizeof(T));

		slot.sequence.store(2 * index + 1, std::memory_order_relaxed);
		std::atomic_thread_fence(std::memory_order_release);
		for (std::size_t i = 0; i < kWords; i++)
			slot.words[i].store(words[i], std::memory_order_relaxed);
		slot.sequence.store(2 * index + 2, std::memory_order_release);
		head_.store(index + 1, std::memory_order_release);
	}

	// The newest samples, up to maxSamples and the capacity, oldest first.
	std::size_t snapshot(std::vector<T>& out, std::size_t maxSamples) const {
		out.clear();
		const std::uint64_t head = head_.load(std::memory_order_acquire);
		const std::uint64_t wanted = std::min<std::uint64_t>({head, maxSamples, capacity_});
		out.reserve(static_cast<std::size_t>(wanted));
		for (std::uint64_t index = head - wanted; index < head; index++) {
			T sample;
			if (read(index, sample))
				out.push_back(sample);
		}
		return out.size();
	}

private:
	bool read(std::uint64_t index, T& sample) const {
		const Slot& slot = slots_[index % capacity_];
		const std::uint64_t expected = 2 * index + 2;
		if (slot.sequence.load(std::memory_order_acquire) != expected)
			return false;
		std::uint64_t words[kWords];
		for (std::size_t i = 0; i < kWords; i++)
			words[i] = slot.words[i].load(std::memory_order_relaxed);
		std::atomic_thread_fence(std::memory_order_acquire);
		if (slot.sequence.load(std::memory_order_relaxed) != expected)
			return false;
		std::memcpy(&sample, words, sizeof(T));
		return true;
	}

	std::size_t capacity_;
	std::unique_ptr<Slot[]> slots_;
	std::atomic<std::uint64_t> head_{0};
};

}  // namespace kelo

#endif  // KELOTULIP_SEQLOCKRING_H
