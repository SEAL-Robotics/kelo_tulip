#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "kelo_tulip/RtLog.h"

using kelo::RtLog;

namespace {

struct Collector {
	std::vector<std::string> lines;
	RtLog::Sink sink() {
		return [this](const char* text) { lines.push_back(text); };
	}
};

TEST(RtLog, drainEmitsLinesInOrder) {
	Collector out;
	RtLog log(out.sink(), 8);
	log.push("one");
	log.push("two");

	EXPECT_EQ(log.drain(), 2u);
	ASSERT_EQ(out.lines.size(), 2u);
	EXPECT_EQ(out.lines[0], "one");
	EXPECT_EQ(out.lines[1], "two");
	EXPECT_EQ(log.drain(), 0u);
}

TEST(RtLog, pushfFormats) {
	Collector out;
	RtLog log(out.sink(), 8);
	log.pushf("wheel %d status1=%d", 3, 61);
	log.drain();
	ASSERT_EQ(out.lines.size(), 1u);
	EXPECT_EQ(out.lines[0], "wheel 3 status1=61");
}

TEST(RtLog, longLinesAreTruncatedNotOverrun) {
	Collector out;
	RtLog log(out.sink(), 8);
	log.push(std::string(1000, 'x').c_str());
	log.drain();
	ASSERT_EQ(out.lines.size(), 1u);
	EXPECT_EQ(out.lines[0].size(), RtLog::kLineBytes - 1);
}

TEST(RtLog, aFullRingDropsTheNewestAndSaysSo) {
	Collector out;
	RtLog log(out.sink(), 4);
	for (int i = 0; i < 10; i++)
		log.pushf("line %d", i);

	EXPECT_EQ(log.dropped(), 6u);
	log.drain();
	ASSERT_GE(out.lines.size(), 4u);
	EXPECT_EQ(out.lines[0], "line 0");
	EXPECT_EQ(out.lines[3], "line 3");
	EXPECT_NE(out.lines.back().find("dropped"), std::string::npos) << out.lines.back();
}

TEST(RtLog, theDrainThreadEmitsWithoutAnyoneCallingDrain) {
	Collector out;
	std::size_t seen = 0;
	{
		RtLog log(out.sink(), 8);
		log.start();
		log.push("from the loop");
		for (int i = 0; i < 200; i++) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			log.stop();
			break;
		}
	}
	seen = out.lines.size();
	EXPECT_EQ(seen, 1u);
}

}  // namespace
