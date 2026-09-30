#include "kelo_tulip/DumpPolicy.h"

#include <algorithm>

namespace kelo {

const char* dumpReasonName(DumpReason reason) {
	switch (reason) {
		case DumpReason::WkcError: return "wkc_error";
		case DumpReason::SlaveLost: return "slave_lost";
		case DumpReason::WheelRecovery: return "wheel_recovery";
		case DumpReason::WheelFailed: return "wheel_failed";
	}
	return "unknown";
}

bool TriggerRateLimiter::tryAccept(std::int64_t nowNs) {
	if (hasAccepted_ && nowNs - lastAcceptedNs_ < minIntervalNs_)
		return false;
	hasAccepted_ = true;
	lastAcceptedNs_ = nowNs;
	return true;
}

std::vector<std::string> dumpFilesToDelete(std::vector<DumpFile> existing, std::size_t maxFiles,
	std::uint64_t maxTotalBytes, std::uint64_t incomingBytes) {
	std::sort(existing.begin(), existing.end(),
		[](const DumpFile& a, const DumpFile& b) {
			return a.order != b.order ? a.order < b.order : a.name < b.name;
		});
	std::uint64_t total = incomingBytes;
	for (const DumpFile& f : existing)
		total += f.bytes;
	// The incoming file counts against the file budget too, and always stays.
	std::size_t count = existing.size() + 1;
	const std::size_t fileBudget = std::max<std::size_t>(maxFiles, 1);

	std::vector<std::string> doomed;
	for (const DumpFile& f : existing) {
		if (count <= fileBudget && total <= maxTotalBytes)
			break;
		doomed.push_back(f.name);
		count--;
		total -= f.bytes;
	}
	return doomed;
}

}  // namespace kelo
