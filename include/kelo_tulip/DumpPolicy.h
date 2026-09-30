/******************************************************************************
 * This software is published under the same dual-license as the rest of
 * kelo_tulip: GNU Lesser General Public License LGPL 2.1 and BSD license.
 ******************************************************************************/

#ifndef KELOTULIP_DUMPPOLICY_H
#define KELOTULIP_DUMPPOLICY_H

#include <cstdint>
#include <string>
#include <vector>

namespace kelo {

enum class DumpReason {
	WkcError,
	SlaveLost,
	WheelRecovery,
	WheelFailed
};

// A wheel giving up ends in a stack restart within tens of milliseconds, so it
// is the one event that must never be swallowed by the rate limit.
inline bool isCriticalDumpReason(DumpReason reason) {
	return reason == DumpReason::WheelFailed;
}

const char* dumpReasonName(DumpReason reason);

// Keeps the first trigger of a burst and drops the rest until the interval passed.
class TriggerRateLimiter {
public:
	explicit TriggerRateLimiter(std::int64_t minIntervalNs) : minIntervalNs_(minIntervalNs) {}
	bool tryAccept(std::int64_t nowNs);

private:
	std::int64_t minIntervalNs_;
	std::int64_t lastAcceptedNs_ = 0;
	bool hasAccepted_ = false;
};

struct DumpFile {
	std::string name;
	std::uint64_t bytes;
	// Age: modification time in ns; ties fall back to the name.
	std::int64_t order = 0;
};

// The oldest files (by order, then name) to delete so that, with one more file of incomingBytes, at
// most maxFiles files and maxTotalBytes bytes remain.
std::vector<std::string> dumpFilesToDelete(std::vector<DumpFile> existing, std::size_t maxFiles,
	std::uint64_t maxTotalBytes, std::uint64_t incomingBytes);

}  // namespace kelo

#endif  // KELOTULIP_DUMPPOLICY_H
