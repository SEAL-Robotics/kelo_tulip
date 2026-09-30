#include "kelo_tulip/EthercatBlackBox.h"

#include <algorithm>
#include <cerrno>
#include <dirent.h>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <chrono>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <ostream>
#include <sstream>

namespace fs = std::filesystem;

namespace kelo {

namespace {

constexpr std::int64_t kNsPerS = 1000000000LL;
constexpr auto kPollPeriod = std::chrono::milliseconds(20);
constexpr auto kSleepSlice = std::chrono::milliseconds(10);
// The ring must outlast the post-trigger wait plus the copy.
constexpr std::size_t kRingSlack = 256;
const char kFilePrefix[] = "ecat_";
const char kFileSuffix[] = ".csv";
const char kPartSuffix[] = ".part";

std::int64_t steadyNowNs() {
	return std::chrono::duration_cast<std::chrono::nanoseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string wallClockStamp() {
	const std::time_t now = std::time(nullptr);
	std::tm local;
	localtime_r(&now, &local);
	char text[32];
	std::strftime(text, sizeof(text), "%Y%m%d-%H%M%S", &local);
	return text;
}

std::size_t ringCapacity(const BlackBoxConfig& config) {
	return static_cast<std::size_t>(config.historyS * config.cycleHz) + kRingSlack;
}

bool endsWith(const std::string& text, const std::string& suffix) {
	return text.size() >= suffix.size() && text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// Finished dumps and the leftovers of an interrupted write.
bool isDumpName(const std::string& name) {
	return name.rfind(kFilePrefix, 0) == 0 && (endsWith(name, kFileSuffix) || endsWith(name, kPartSuffix));
}

}  // namespace

void writeBlackBoxCsv(std::ostream& out, const std::vector<CycleSample>& samples, std::size_t wheelCount,
	std::uint64_t triggerCycle) {
	wheelCount = std::min(wheelCount, kMaxBlackBoxWheels);
	out << "t_ms,cycle,wkc,expected_wkc,driver_state,trigger";
	for (std::size_t w = 0; w < wheelCount; w++) {
		out << ",w" << w << "_sensor_ts,w" << w << "_voltage_bus,w" << w << "_current_in,w" << w << "_status1,w"
			<< w << "_status2,w" << w << "_command1,w" << w << "_iq1,w" << w << "_iq2,w" << w << "_setpoint1,w"
			<< w << "_setpoint2,w" << w << "_limit1_p,w" << w << "_limit2_p,w" << w << "_wheel_state,w" << w
			<< "_link_up,w" << w << "_enabled";
	}
	out << '\n';

	// A trigger older than the history has no row: time then counts back from
	// the last sample.
	std::int64_t triggerNs = samples.empty() ? 0 : samples.back().monotonicNs;
	for (const CycleSample& s : samples)
		if (s.cycle == triggerCycle)
			triggerNs = s.monotonicNs;

	out << std::setprecision(7);
	for (const CycleSample& s : samples) {
		const double tMs = static_cast<double>(s.monotonicNs - triggerNs) / 1.0e6;
		out << tMs << ',' << s.cycle << ',' << s.wkc << ',' << s.expectedWkc << ',' << static_cast<int>(s.driverState)
			<< ',' << (s.cycle == triggerCycle ? 1 : 0);
		for (std::size_t w = 0; w < wheelCount; w++) {
			const WheelSample& ws = s.wheels[w];
			out << ',' << ws.sensorTs << ',' << ws.voltageBus << ',' << ws.currentIn << ',' << ws.status1 << ','
				<< ws.status2 << ',' << ws.command1 << ',' << ws.current1q << ',' << ws.current2q << ','
				<< ws.setpoint1 << ',' << ws.setpoint2 << ',' << ws.limit1p << ',' << ws.limit2p << ','
				<< static_cast<int>(ws.wheelState) << ',' << ((ws.flags & kWheelFlagLinkUp) ? 1 : 0) << ','
				<< ((ws.flags & kWheelFlagEnabled) ? 1 : 0);
		}
		out << '\n';
	}
}

EthercatBlackBox::EthercatBlackBox(const BlackBoxConfig& config)
	: config_(config)
	, ring_(ringCapacity(config))
{
	enabled_ = !config_.dir.empty();
	if (!enabled_) {
		std::cerr << "[ecat-blackbox] ERROR no dump directory: black box is off" << std::endl;
		return;
	}
	writer_ = std::thread(&EthercatBlackBox::writerLoop, this);
}

EthercatBlackBox::~EthercatBlackBox() {
	stopping_ = true;
	if (writer_.joinable())
		writer_.join();
}

void EthercatBlackBox::record(const CycleSample& sample) {
	if (!enabled_)
		return;
	ring_.push(sample);
	lastCycle_.store(sample.cycle, std::memory_order_release);
}

void EthercatBlackBox::setBusStatus(int wkc, int expectedWkc) {
	wkc_.store(wkc, std::memory_order_relaxed);
	expectedWkc_.store(expectedWkc, std::memory_order_relaxed);
}

void EthercatBlackBox::trigger(DumpReason reason) {
	if (!enabled_)
		return;
	post(isCriticalDumpReason(reason) ? critical_ : normal_, reason);
}

void EthercatBlackBox::post(Slot& slot, DumpReason reason) {
	int expected = 0;
	if (!slot.state.compare_exchange_strong(expected, 1, std::memory_order_acquire)) {
		triggersDropped_++;
		return;
	}
	slot.reason.store(static_cast<int>(reason), std::memory_order_relaxed);
	slot.cycle.store(lastCycle_.load(std::memory_order_acquire), std::memory_order_relaxed);
	slot.state.store(2, std::memory_order_release);
}

bool EthercatBlackBox::takeTrigger(Slot& slot, TriggerRateLimiter& limiter, DumpReason& reason,
	std::uint64_t& cycle) {
	if (slot.state.load(std::memory_order_acquire) != 2)
		return false;
	reason = static_cast<DumpReason>(slot.reason.load(std::memory_order_relaxed));
	cycle = slot.cycle.load(std::memory_order_relaxed);
	slot.state.store(0, std::memory_order_release);
	if (!limiter.tryAccept(steadyNowNs())) {
		triggersDropped_++;
		return false;
	}
	return true;
}

void EthercatBlackBox::sleepUnlessStopping(double seconds) {
	const auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
	while (!stopping_ && std::chrono::steady_clock::now() < until)
		std::this_thread::sleep_for(kSleepSlice);
}

void EthercatBlackBox::writerLoop() {
	const std::int64_t intervalNs = static_cast<std::int64_t>(config_.minIntervalS * kNsPerS);
	TriggerRateLimiter normalLimiter(intervalNs);
	// A wheel giving up is never rate limited: the disk budget bounds it.
	TriggerRateLimiter criticalLimiter(0);
	for (;;) {
		const bool stopping = stopping_;
		DumpReason reason;
		std::uint64_t cycle = 0;
		// A wheel giving up ends in a restart, so it goes first and does not wait.
		try {
			if (takeTrigger(critical_, criticalLimiter, reason, cycle)) {
				writeDump(reason, cycle);
				continue;
			}
			if (takeTrigger(normal_, normalLimiter, reason, cycle)) {
				sleepUnlessStopping(config_.postTriggerS);
				writeDump(reason, cycle);
				continue;
			}
		} catch (...) {
			std::cerr << "[ecat-blackbox] unexpected error, dump skipped" << std::endl;
			continue;
		}
		if (stopping)
			return;
		std::this_thread::sleep_for(kPollPeriod);
	}
}

void EthercatBlackBox::writeDump(DumpReason reason, std::uint64_t triggerCycle) {
	// A full disk or an allocation failure must cost the dump, not the driver.
	try {
		writeDumpUnchecked(reason, triggerCycle);
	} catch (const std::exception& e) {
		std::cerr << "[ecat-blackbox] dump failed: " << e.what() << std::endl;
	}
}

void EthercatBlackBox::writeDumpUnchecked(DumpReason reason, std::uint64_t triggerCycle) {
	std::vector<CycleSample> samples;
	ring_.snapshot(samples, ring_.capacity() > kRingSlack ? ring_.capacity() - kRingSlack : ring_.capacity());
	if (samples.empty())
		return;

	std::ostringstream body;
	writeBlackBoxCsv(body, samples, samples.back().wheelCount, triggerCycle);
	const std::string text = body.str();

	if (text.size() > config_.maxTotalBytes) {
		std::cerr << "[ecat-blackbox] a dump of " << text.size() / 1024 << " KiB does not fit the "
			<< config_.maxTotalBytes / 1024 << " KiB budget, skipped" << std::endl;
		return;
	}

	const int dirFd = openDumpDirectory();
	if (dirFd < 0)
		return;
	pruneDirectory(dirFd, text.size());
	const std::string name = createExclusive(dirFd, reason, text);
	::close(dirFd);
	if (name.empty())
		return;
	dumpsWritten_++;
	std::cout << "[ecat-blackbox] wrote " << (fs::path(config_.dir) / name).string() << " (" << text.size() / 1024
		<< " KiB, " << dumpReasonName(reason) << ")" << std::endl;
}

// Every later operation goes through this descriptor, so a directory swapped
// for a link afterwards is not followed. The directory must be ours and closed
// to group and others, or dumping is refused.
int EthercatBlackBox::openDumpDirectory() {
	if (refused_)
		return -1;
	std::error_code ec;
	const fs::path dir(config_.dir);
	if (dir.has_parent_path())
		fs::create_directories(dir.parent_path(), ec);
	if (::mkdir(config_.dir.c_str(), 0700) != 0 && errno != EEXIST) {
		std::cerr << "[ecat-blackbox] ERROR cannot create " << config_.dir << ": " << std::strerror(errno)
			<< std::endl;
		return -1;
	}
	const int fd = ::open(config_.dir.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
	struct stat st;
	if (fd < 0 || ::fstat(fd, &st) != 0 || st.st_uid != ::geteuid() || (st.st_mode & (S_IWGRP | S_IWOTH)) != 0) {
		std::cerr << "[ecat-blackbox] ERROR " << config_.dir
			<< " is a link, not ours, or writable by others: black box dumps are disabled" << std::endl;
		if (fd >= 0)
			::close(fd);
		refused_ = true;
		return -1;
	}
	return fd;
}

void EthercatBlackBox::pruneDirectory(int dirFd, std::uint64_t incomingBytes) {
	std::vector<DumpFile> existing;
	DIR* dir = ::fdopendir(::dup(dirFd));
	if (dir) {
		while (const dirent* entry = ::readdir(dir)) {
			const std::string name = entry->d_name;
			struct stat st;
			if (!isDumpName(name) || ::fstatat(dirFd, name.c_str(), &st, AT_SYMLINK_NOFOLLOW) != 0 ||
				!S_ISREG(st.st_mode))
				continue;
			const std::int64_t mtimeNs = static_cast<std::int64_t>(st.st_mtim.tv_sec) * kNsPerS + st.st_mtim.tv_nsec;
			existing.push_back(DumpFile{name, static_cast<std::uint64_t>(st.st_size), mtimeNs});
		}
		::closedir(dir);
	}
	for (const std::string& name : dumpFilesToDelete(existing, config_.maxFiles, config_.maxTotalBytes, incomingBytes))
		::unlinkat(dirFd, name.c_str(), 0);
}

// Written under a .part name, synced, renamed, and the directory synced. O_EXCL
// and O_NOFOLLOW mean an existing name or a planted link is skipped, never
// written through. Returns the file name, or "" on failure.
std::string EthercatBlackBox::createExclusive(int dirFd, DumpReason reason, const std::string& text) {
	constexpr int kNameAttempts = 8;
	for (int attempt = 0; attempt < kNameAttempts; attempt++) {
		std::ostringstream stream;
		stream << kFilePrefix << wallClockStamp() << '_' << std::setfill('0') << std::setw(4) << fileSequence_++
			<< '_' << dumpReasonName(reason) << kFileSuffix;
		const std::string name = stream.str();
		const std::string part = name + kPartSuffix;
		const int fd = ::openat(dirFd, part.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600);
		if (fd < 0) {
			if (errno == EEXIST)
				continue;
			std::cerr << "[ecat-blackbox] cannot write " << part << ": " << std::strerror(errno) << std::endl;
			return "";
		}
		bool ok = true;
		for (std::size_t done = 0; ok && done < text.size();) {
			const ssize_t n = ::write(fd, text.data() + done, text.size() - done);
			if (n < 0 && errno == EINTR)
				continue;
			ok = n > 0;
			done += ok ? static_cast<std::size_t>(n) : 0;
		}
		ok = ok && ::fsync(fd) == 0;
		::close(fd);
		if (!ok || ::renameat(dirFd, part.c_str(), dirFd, name.c_str()) != 0) {
			std::cerr << "[ecat-blackbox] cannot finish " << name << ": " << std::strerror(errno) << std::endl;
			::unlinkat(dirFd, part.c_str(), 0);
			return "";
		}
		::fsync(dirFd);
		return name;
	}
	std::cerr << "[ecat-blackbox] no free dump name in " << config_.dir << std::endl;
	return "";
}

}  // namespace kelo
