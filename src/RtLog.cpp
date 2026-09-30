#include "kelo_tulip/RtLog.h"

#include <algorithm>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

namespace kelo {

namespace {
constexpr auto kDrainPeriod = std::chrono::milliseconds(20);

void writeToStdout(const char* text) {
	std::cout << text << std::endl;
}
}  // namespace

RtLog::RtLog(Sink sink, std::size_t slots)
	: sink_(sink ? sink : Sink(writeToStdout))
	, slots_(std::max<std::size_t>(slots, 1))
	, ring_(new Slot[slots_])
{
}

RtLog::~RtLog() {
	stop();
	drain();
}

RtLog& RtLog::instance() {
	static RtLog log;
	return log;
}

void RtLog::push(const char* text) {
	const std::uint64_t head = head_.load(std::memory_order_relaxed);
	if (head - tail_.load(std::memory_order_acquire) >= slots_) {
		dropped_.fetch_add(1, std::memory_order_relaxed);
		return;
	}
	Slot& slot = ring_[head % slots_];
	std::strncpy(slot.text, text, kLineBytes - 1);
	slot.text[kLineBytes - 1] = '\0';
	head_.store(head + 1, std::memory_order_release);
}

void RtLog::pushf(const char* format, ...) {
	char text[kLineBytes];
	va_list args;
	va_start(args, format);
	std::vsnprintf(text, sizeof(text), format, args);
	va_end(args);
	push(text);
}

std::size_t RtLog::drain() {
	std::size_t emitted = 0;
	const std::uint64_t head = head_.load(std::memory_order_acquire);
	std::uint64_t tail = tail_.load(std::memory_order_relaxed);
	for (; tail < head; tail++) {
		sink_(ring_[tail % slots_].text);
		emitted++;
	}
	tail_.store(tail, std::memory_order_release);
	const std::uint64_t dropped = dropped_.load(std::memory_order_relaxed);
	if (dropped != reportedDropped_) {
		sink_(("[rt-log] " + std::to_string(dropped - reportedDropped_) + " lines dropped").c_str());
		reportedDropped_ = dropped;
	}
	return emitted;
}

void RtLog::start() {
	if (thread_.joinable())
		return;
	stopRequested_ = false;
	thread_ = std::thread([this] {
		while (!stopRequested_) {
			drain();
			std::this_thread::sleep_for(kDrainPeriod);
		}
	});
}

void RtLog::stop() {
	stopRequested_ = true;
	if (thread_.joinable())
		thread_.join();
}

}  // namespace kelo
