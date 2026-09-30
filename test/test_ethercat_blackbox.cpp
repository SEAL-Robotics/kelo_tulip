#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <unistd.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <thread>

#include "kelo_tulip/EthercatBlackBox.h"

namespace fs = std::filesystem;
using kelo::BlackBoxConfig;
using kelo::CycleSample;
using kelo::DumpReason;
using kelo::EthercatBlackBox;

namespace {

CycleSample makeSample(std::uint64_t cycle) {
	CycleSample s{};
	s.cycle = cycle;
	s.monotonicNs = static_cast<std::int64_t>(cycle) * 1000000;  // 1 kHz
	s.wkc = 12;
	s.expectedWkc = 12;
	s.wheelCount = 2;
	for (int w = 0; w < 2; w++) {
		s.wheels[w].sensorTs = cycle * 1000;
		s.wheels[w].voltageBus = 51.5f;
		s.wheels[w].currentIn = 1.25f;
		s.wheels[w].status1 = 63;
		s.wheels[w].status2 = 2051;
	}
	return s;
}

std::vector<CycleSample> samples(std::uint64_t first, std::uint64_t count) {
	std::vector<CycleSample> v;
	for (std::uint64_t i = 0; i < count; i++)
		v.push_back(makeSample(first + i));
	return v;
}

std::vector<std::string> lines(const std::string& text) {
	std::vector<std::string> out;
	std::istringstream in(text);
	for (std::string line; std::getline(in, line);)
		out.push_back(line);
	return out;
}

std::size_t commas(const std::string& s) {
	return static_cast<std::size_t>(std::count(s.begin(), s.end(), ','));
}

std::size_t csvCount(const fs::path& dir) {
	std::size_t n = 0;
	if (!fs::exists(dir))
		return 0;
	for (const auto& e : fs::directory_iterator(dir))
		if (e.path().extension() == ".csv")
			n++;
	return n;
}

fs::path firstCsv(const fs::path& dir) {
	for (const auto& e : fs::directory_iterator(dir))
		if (e.path().extension() == ".csv")
			return e.path();
	return {};
}

bool waitFor(const std::function<bool()>& cond, int timeoutMs = 5000) {
	for (int waited = 0; waited < timeoutMs; waited += 10) {
		if (cond())
			return true;
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	return cond();
}

class TempDir {
public:
	TempDir() {
		path_ = fs::temp_directory_path() / ("ecat_blackbox_test_" + std::to_string(::getpid()) + "_" +
			std::to_string(counter_++));
		fs::remove_all(path_);
	}
	~TempDir() { fs::remove_all(path_); }
	const fs::path& path() const { return path_; }

private:
	fs::path path_;
	static inline int counter_ = 0;
};

BlackBoxConfig testConfig(const fs::path& dir) {
	BlackBoxConfig c;
	c.dir = dir.string();
	c.historyS = 1.0;
	c.postTriggerS = 0.05;
	c.minIntervalS = 10.0;
	return c;
}

TEST(SanitizeBlackBoxConfig, clampsEveryBoundedValue) {
	BlackBoxConfig c;
	c.historyS = 1000.0;
	c.postTriggerS = 500.0;
	c.minIntervalS = -1.0;
	c.maxFiles = 0;
	c.maxTotalBytes = 0;
	c.cycleHz = 0.0;

	const BlackBoxConfig s = kelo::sanitizeBlackBoxConfig(c);

	EXPECT_LE(s.historyS, 30.0);
	EXPECT_LE(s.postTriggerS, s.historyS / 2.0);
	EXPECT_GE(s.minIntervalS, 0.0);
	EXPECT_GE(s.maxFiles, 1u);
	EXPECT_GE(s.maxTotalBytes, 1024u * 1024u);
	EXPECT_GT(s.cycleHz, 0.0);
}

TEST(SanitizeBlackBoxConfig, leavesSaneValuesAlone) {
	BlackBoxConfig c;
	const BlackBoxConfig s = kelo::sanitizeBlackBoxConfig(c);
	EXPECT_EQ(s.historyS, c.historyS);
	EXPECT_EQ(s.postTriggerS, c.postTriggerS);
	EXPECT_EQ(s.minIntervalS, c.minIntervalS);
	EXPECT_EQ(s.maxFiles, c.maxFiles);
	EXPECT_EQ(s.maxTotalBytes, c.maxTotalBytes);
}

TEST(WriteBlackBoxCsv, hasOneColumnCountPerRowAndMarksTheTriggerRow) {
	std::ostringstream out;
	kelo::writeBlackBoxCsv(out, samples(100, 5), 2, 102);

	const auto rows = lines(out.str());
	ASSERT_EQ(rows.size(), 6u);
	for (const auto& row : rows)
		EXPECT_EQ(commas(row), commas(rows[0])) << row;
	EXPECT_EQ(rows[0].rfind("t_ms,cycle,wkc,expected_wkc,driver_state,trigger,", 0), 0u) << rows[0];
	EXPECT_NE(rows[0].find("w1_voltage_bus"), std::string::npos);
	EXPECT_EQ(rows[0].find("w2_voltage_bus"), std::string::npos);
}

TEST(WriteBlackBoxCsv, timeIsRelativeToTheTriggerSample) {
	std::ostringstream out;
	kelo::writeBlackBoxCsv(out, samples(100, 5), 2, 102);

	const auto rows = lines(out.str());
	ASSERT_EQ(rows.size(), 6u);
	EXPECT_EQ(rows[1].rfind("-2,100,12,12,", 0), 0u) << rows[1];
	EXPECT_EQ(rows[3].rfind("0,102,12,12,0,1,", 0), 0u) << rows[3];
	EXPECT_EQ(rows[5].rfind("2,104,12,12,", 0), 0u) << rows[5];
}

TEST(WriteBlackBoxCsv, carriesTheWheelValues) {
	std::ostringstream out;
	kelo::writeBlackBoxCsv(out, samples(100, 1), 2, 100);

	const auto rows = lines(out.str());
	ASSERT_EQ(rows.size(), 2u);
	const std::string& row = rows[1];
	EXPECT_NE(row.find(",100000,51.5,1.25,63,2051,"), std::string::npos) << row;
}

TEST(EthercatBlackBox, aTriggerWritesTheRecentHistoryToACsv) {
	TempDir tmp;
	{
		EthercatBlackBox box(testConfig(tmp.path()));
		for (const auto& s : samples(0, 300))
			box.record(s);
		box.trigger(DumpReason::WkcError);
		for (const auto& s : samples(300, 100))
			box.record(s);
		ASSERT_TRUE(waitFor([&] { return csvCount(tmp.path()) == 1; }));
	}
	const fs::path file = firstCsv(tmp.path());
	EXPECT_NE(file.filename().string().find("wkc_error"), std::string::npos) << file;
	std::ifstream in(file);
	std::stringstream text;
	text << in.rdbuf();
	const auto rows = lines(text.str());
	ASSERT_GT(rows.size(), 300u);
	std::size_t triggerRows = 0;
	for (std::size_t i = 1; i < rows.size(); i++)
		if (rows[i].find(",0,1,") != std::string::npos && rows[i].rfind("0,299,", 0) == 0)
			triggerRows++;
	EXPECT_EQ(triggerRows, 1u) << "the trigger row is the last sample recorded before trigger()";
}

TEST(EthercatBlackBox, burstsAreRateLimitedKeepingTheFirst) {
	TempDir tmp;
	EthercatBlackBox box(testConfig(tmp.path()));
	for (const auto& s : samples(0, 100))
		box.record(s);
	box.trigger(DumpReason::WkcError);
	ASSERT_TRUE(waitFor([&] { return csvCount(tmp.path()) == 1; }));
	box.trigger(DumpReason::SlaveLost);
	box.trigger(DumpReason::WheelRecovery);
	std::this_thread::sleep_for(std::chrono::milliseconds(300));

	EXPECT_EQ(csvCount(tmp.path()), 1u);
	EXPECT_NE(firstCsv(tmp.path()).filename().string().find("wkc_error"), std::string::npos);
	EXPECT_GE(box.triggersDropped(), 1u);
}

TEST(EthercatBlackBox, aWheelFailureIsNotSwallowedByTheRateLimit) {
	TempDir tmp;
	EthercatBlackBox box(testConfig(tmp.path()));
	for (const auto& s : samples(0, 100))
		box.record(s);
	box.trigger(DumpReason::WkcError);
	ASSERT_TRUE(waitFor([&] { return csvCount(tmp.path()) == 1; }));
	box.trigger(DumpReason::WheelFailed);

	EXPECT_TRUE(waitFor([&] { return csvCount(tmp.path()) == 2; }));
}

TEST(EthercatBlackBox, criticalDumpsAreNotRateLimited) {
	TempDir tmp;
	EthercatBlackBox box(testConfig(tmp.path()));
	for (const auto& s : samples(0, 100))
		box.record(s);
	box.trigger(DumpReason::WheelFailed);
	ASSERT_TRUE(waitFor([&] { return csvCount(tmp.path()) == 1; }));
	box.trigger(DumpReason::WheelFailed);

	EXPECT_TRUE(waitFor([&] { return csvCount(tmp.path()) == 2; }));
}

TEST(EthercatBlackBox, dumpsAreOwnerOnly) {
	TempDir tmp;
	{
		EthercatBlackBox box(testConfig(tmp.path()));
		for (const auto& s : samples(0, 100))
			box.record(s);
		box.trigger(DumpReason::WheelFailed);
		ASSERT_TRUE(waitFor([&] { return csvCount(tmp.path()) == 1; }));
	}
	const auto file = firstCsv(tmp.path());
	EXPECT_EQ(fs::status(file).permissions() & (fs::perms::group_all | fs::perms::others_all), fs::perms::none);
	EXPECT_EQ(fs::status(tmp.path()).permissions() & (fs::perms::group_all | fs::perms::others_all),
		fs::perms::none);
}

TEST(WriteBlackBoxCsv, aTriggerThatAgedOutOfTheHistoryFallsBackToTheLastSample) {
	std::ostringstream out;
	kelo::writeBlackBoxCsv(out, samples(100, 3), 2, 5);

	const auto rows = lines(out.str());
	ASSERT_EQ(rows.size(), 4u);
	EXPECT_EQ(rows[3].rfind("0,102,", 0), 0u) << rows[3];
	EXPECT_EQ(rows[1].rfind("-2,100,", 0), 0u) << rows[1];
}

TEST(EthercatBlackBox, aDumpDirectoryOthersCanWriteIsRefused) {
	TempDir tmp;
	fs::create_directories(tmp.path());
	fs::permissions(tmp.path(), fs::perms::all);
	{
		EthercatBlackBox box(testConfig(tmp.path()));
		for (const auto& s : samples(0, 100))
			box.record(s);
		box.trigger(DumpReason::WheelFailed);
		std::this_thread::sleep_for(std::chrono::milliseconds(300));
	}
	EXPECT_EQ(csvCount(tmp.path()), 0u);
}

TEST(EthercatBlackBox, aSymlinkedDumpDirectoryIsRefused) {
	TempDir tmp;
	TempDir target;
	fs::create_directories(target.path());
	fs::permissions(target.path(), fs::perms::owner_all);
	fs::create_directory_symlink(target.path(), tmp.path());
	{
		EthercatBlackBox box(testConfig(tmp.path()));
		for (const auto& s : samples(0, 100))
			box.record(s);
		box.trigger(DumpReason::WheelFailed);
		std::this_thread::sleep_for(std::chrono::milliseconds(300));
	}
	EXPECT_EQ(csvCount(target.path()), 0u);
	fs::remove(tmp.path());
}

TEST(EthercatBlackBox, aDumpLargerThanTheWholeBudgetIsSkipped) {
	TempDir tmp;
	BlackBoxConfig cfg = testConfig(tmp.path());
	cfg.maxTotalBytes = 1000;
	{
		EthercatBlackBox box(cfg);
		for (const auto& s : samples(0, 100))
			box.record(s);
		box.trigger(DumpReason::WheelFailed);
		std::this_thread::sleep_for(std::chrono::milliseconds(300));
	}
	EXPECT_EQ(csvCount(tmp.path()), 0u);
}

TEST(EthercatBlackBox, anEmptyDirectoryDisablesItOnceAndQuietly) {
	EthercatBlackBox box(testConfig(fs::path()));
	EXPECT_FALSE(box.enabled());
	for (const auto& s : samples(0, 10))
		box.record(s);
	box.trigger(DumpReason::WheelFailed);
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	EXPECT_EQ(box.dumpsWritten(), 0u);
}

TEST(EthercatBlackBox, destroyingTheBoxStillWritesAPendingCriticalDump) {
	TempDir tmp;
	BlackBoxConfig cfg = testConfig(tmp.path());
	cfg.postTriggerS = 30.0;  // would never finish waiting
	cfg.historyS = 60.0;
	{
		EthercatBlackBox box(cfg);
		for (const auto& s : samples(0, 100))
			box.record(s);
		box.trigger(DumpReason::WheelFailed);
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	EXPECT_EQ(csvCount(tmp.path()), 1u);
}

TEST(EthercatBlackBox, diskUseStaysWithinTheFileBudget) {
	TempDir tmp;
	BlackBoxConfig cfg = testConfig(tmp.path());
	cfg.minIntervalS = 0.0;
	cfg.postTriggerS = 0.0;
	cfg.maxFiles = 3;
	EthercatBlackBox box(cfg);
	for (const auto& s : samples(0, 50))
		box.record(s);
	for (int i = 0; i < 8; i++) {
		const std::uint64_t before = box.dumpsWritten();
		box.trigger(DumpReason::WkcError);
		ASSERT_TRUE(waitFor([&] { return box.dumpsWritten() > before; }));
	}

	EXPECT_LE(csvCount(tmp.path()), 3u);
	EXPECT_GE(csvCount(tmp.path()), 1u);
}

TEST(EthercatBlackBox, recordingAndTriggeringWithoutAnyoneWaitingIsSafe) {
	TempDir tmp;
	EthercatBlackBox box(testConfig(tmp.path()));
	for (const auto& s : samples(0, 5000))
		box.record(s);
	box.trigger(DumpReason::SlaveLost);
	box.setBusStatus(-1, 12);
	EXPECT_EQ(box.wkc(), -1);
	EXPECT_EQ(box.expectedWkc(), 12);
}

}  // namespace
