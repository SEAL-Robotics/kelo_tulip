/******************************************************************************
 * This software is published under the same dual-license as the rest of
 * kelo_tulip: GNU Lesser General Public License LGPL 2.1 and BSD license.
 ******************************************************************************/

#ifndef KELOTULIP_ETHERCATBLACKBOX_H
#define KELOTULIP_ETHERCATBLACKBOX_H

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include "kelo_tulip/BlackBoxSample.h"
#include "kelo_tulip/DumpPolicy.h"
#include "kelo_tulip/SeqlockRing.h"

namespace kelo {

struct BlackBoxConfig {
	std::string dir;
	double historyS = 2.0;
	double cycleHz = 1000.0;
	// Time kept recording after the trigger before the file is written.
	double postTriggerS = 0.5;
	double minIntervalS = 10.0;
	std::size_t maxFiles = 100;
	std::uint64_t maxTotalBytes = 256ull * 1024 * 1024;
};

// Parameters come from a file: keep the ring, the wait and the disk use within
// sane bounds whatever it says.
inline BlackBoxConfig sanitizeBlackBoxConfig(BlackBoxConfig config) {
	constexpr double kMinHistoryS = 0.5;
	// A dump of the longest history must fit the smallest budget.
	constexpr double kMaxHistoryS = 15.0;
	constexpr std::uint64_t kMinTotalBytes = 16ull * 1024ull * 1024ull;
	if (!(config.cycleHz > 0.0))
		config.cycleHz = 1000.0;
	config.historyS = std::min(std::max(config.historyS, kMinHistoryS), kMaxHistoryS);
	// The wait must leave most of the history from before the trigger.
	config.postTriggerS = std::min(std::max(config.postTriggerS, 0.0), config.historyS / 2.0);
	config.minIntervalS = std::max(config.minIntervalS, 0.0);
	config.maxFiles = std::max<std::size_t>(config.maxFiles, 1);
	config.maxTotalBytes = std::max(config.maxTotalBytes, kMinTotalBytes);
	return config;
}

// A rolling record of the last few seconds of the EtherCAT loop. The loop
// only ever calls record() and trigger(), which copy and set atomics; a writer
// thread does the allocating and the file I/O.
class EthercatBlackBox {
public:
	explicit EthercatBlackBox(const BlackBoxConfig& config);
	~EthercatBlackBox();

	// Realtime thread only.
	void record(const CycleSample& sample);
	// Any thread; wait-free. At most one pending trigger per class is kept.
	void trigger(DumpReason reason);

	void setBusStatus(int wkc, int expectedWkc);
	int wkc() const { return wkc_.load(std::memory_order_relaxed); }
	int expectedWkc() const { return expectedWkc_.load(std::memory_order_relaxed); }

	// False when there is nowhere to dump: nothing is recorded or written.
	bool enabled() const { return enabled_; }

	std::uint64_t dumpsWritten() const { return dumpsWritten_.load(); }
	std::uint64_t triggersDropped() const { return triggersDropped_.load(); }

private:
	// A trigger is claimed, filled in, then published, so the writer never
	// sees a half-written one.
	struct Slot {
		std::atomic<int> state{0};  // 0 free, 1 being filled, 2 ready
		std::atomic<int> reason{0};
		std::atomic<std::uint64_t> cycle{0};
	};

	void post(Slot& slot, DumpReason reason);
	void writerLoop();
	bool takeTrigger(Slot& slot, TriggerRateLimiter& limiter, DumpReason& reason, std::uint64_t& cycle);
	void sleepUnlessStopping(double seconds);
	void writeDump(DumpReason reason, std::uint64_t triggerCycle);
	void writeDumpUnchecked(DumpReason reason, std::uint64_t triggerCycle);
	int openDumpDirectory();
	void pruneDirectory(int dirFd, std::uint64_t incomingBytes);
	std::string createExclusive(int dirFd, DumpReason reason, const std::string& text);

	BlackBoxConfig config_;
	SeqlockRing<CycleSample> ring_;
	std::atomic<std::uint64_t> lastCycle_{0};
	std::atomic<int> wkc_{0};
	std::atomic<int> expectedWkc_{0};
	std::atomic<std::uint64_t> dumpsWritten_{0};
	std::atomic<std::uint64_t> triggersDropped_{0};
	std::atomic<bool> stopping_{false};
	Slot normal_;
	Slot critical_;
	std::uint64_t fileSequence_ = 0;
	bool enabled_ = true;
	bool refused_ = false;  // writer thread only
	std::thread writer_;
};

}  // namespace kelo

#endif  // KELOTULIP_ETHERCATBLACKBOX_H
