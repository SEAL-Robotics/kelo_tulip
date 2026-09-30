#include "kelo_tulip/EscDiagnostics.h"

#include <chrono>
#include <iostream>

namespace kelo {

namespace {

constexpr auto kTick = std::chrono::milliseconds(20);
// A read that gets no answer within this is given up; the loop's own timeout.
constexpr int kReadTimeoutUs = EC_TIMEOUTRET;

}  // namespace

EscDiagnostics::EscDiagnostics(ecx_contextt* context, double periodS)
	: context_(context)
	, periodS_(periodS)
{
}

EscDiagnostics::~EscDiagnostics() {
	stop();
}

void EscDiagnostics::start() {
	std::lock_guard<std::mutex> lock(lifecycle_);
	if (thread_.joinable() || periodS_ <= 0.0)
		return;
	stopRequested_ = false;
	// A first read right away gives the baseline the deltas are taken against.
	pollRequested_ = true;
	thread_ = std::thread(&EscDiagnostics::loop, this);
}

void EscDiagnostics::stop() {
	std::lock_guard<std::mutex> lock(lifecycle_);
	stopRequested_ = true;
	if (thread_.joinable())
		thread_.join();
}

EscSnapshot EscDiagnostics::snapshot() const {
	std::lock_guard<std::mutex> lock(mutex_);
	return latest_;
}

void EscDiagnostics::loop() {
	const auto period = std::chrono::duration<double>(periodS_);
	auto nextPeriodic = std::chrono::steady_clock::now() + period;
	bool inRound = false;
	int nextSlave = 1;
	EscSnapshot round;
	while (!stopRequested_) {
		if (!inRound) {
			const bool requested = pollRequested_.exchange(false);
			const bool periodic = std::chrono::steady_clock::now() >= nextPeriodic;
			if (requested || periodic) {
				inRound = true;
				round = EscSnapshot();
				round.afterEpisode = requested && !periodic;
				nextSlave = 1;
			}
		}
		if (inRound) {
			readSlave(nextSlave++, round);
			if (nextSlave > *context_->slavecount) {
				publish(std::move(round));
				inRound = false;
				nextPeriodic = std::chrono::steady_clock::now() + period;
			}
		}
		std::this_thread::sleep_for(kTick);
	}
}

// The slave list is written by the check thread without synchronisation; a
// read of islost or configadr that races with it is a plain int read and at
// worst yields one failed or skipped read.
void EscDiagnostics::readSlave(int slave, EscSnapshot& round) {
	EscSlaveReading reading;
	reading.slave = slave;
	if (slave <= *context_->slavecount && !context_->slavelist[slave].islost) {
		std::uint8_t raw[kEscErrorRegLength] = {};
		const int wkc = ecx_FPRD(context_->port, context_->slavelist[slave].configadr, kEscErrorRegBase,
			kEscErrorRegLength, raw, kReadTimeoutUs);
		if (wkc == 1) {
			reading.ok = true;
			reading.counters = decodeEscCounters(raw);
			reading.delta = tracker_.update(static_cast<std::size_t>(slave), reading.counters);
			if (reading.delta.any() || reading.delta.restarted) {
				std::cout << "[ecat-diag] slave " << slave << ": " << describeEscDelta(reading.delta)
					<< (reading.delta.restarted ? " (counters restarted)" : "")
					<< (reading.delta.saturated ? " (a counter is saturated at 255)" : "")
					<< (round.afterEpisode ? " [after a communication error]" : "") << std::endl;
			}
		}
	}
	round.slaves.push_back(reading);
}

void EscDiagnostics::publish(EscSnapshot&& round) {
	std::lock_guard<std::mutex> lock(mutex_);
	round.sequence = latest_.sequence + 1;
	latest_ = std::move(round);
	sequence_.store(latest_.sequence, std::memory_order_release);
}

}  // namespace kelo
