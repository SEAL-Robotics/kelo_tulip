/******************************************************************************
 * This software is published under the same dual-license as the rest of
 * kelo_tulip: GNU Lesser General Public License LGPL 2.1 and BSD license.
 ******************************************************************************/

#ifndef KELOTULIP_RTLOG_H
#define KELOTULIP_RTLOG_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <thread>

namespace kelo {

// Log lines from the 1 kHz thread. The thread only formats into a preallocated
// slot; another thread does the stdout write, so a blocked pipe or the stdout
// lock cannot stall the loop. One producer thread, one consumer. A full ring
// drops the newest line and counts it.
class RtLog {
public:
	static constexpr std::size_t kLineBytes = 192;
	using Sink = std::function<void(const char*)>;

	explicit RtLog(Sink sink = nullptr, std::size_t slots = 256);
	~RtLog();

	// The process-wide log the EtherCAT thread writes to.
	static RtLog& instance();

	void push(const char* text);
	// vsnprintf runs on the calling thread: keep to plain numeric and short
	// string conversions (%d %u %.0f %s of a short literal), nothing locale- or
	// allocation-heavy.
	void pushf(const char* format, ...) __attribute__((format(printf, 2, 3)));

	// Emits everything queued; the consumer side.
	std::size_t drain();
	// A drain thread, at most one start() before its stop().
	void start();
	void stop();

	std::uint64_t dropped() const { return dropped_.load(); }

private:
	struct Slot {
		char text[kLineBytes];
	};

	Sink sink_;
	std::size_t slots_;
	std::unique_ptr<Slot[]> ring_;
	std::atomic<std::uint64_t> head_{0};
	std::atomic<std::uint64_t> tail_{0};
	std::atomic<std::uint64_t> dropped_{0};
	std::uint64_t reportedDropped_ = 0;
	std::atomic<bool> stopRequested_{false};
	std::thread thread_;
};

}  // namespace kelo

#endif  // KELOTULIP_RTLOG_H
