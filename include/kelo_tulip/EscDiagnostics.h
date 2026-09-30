/******************************************************************************
 * This software is published under the same dual-license as the rest of
 * kelo_tulip: GNU Lesser General Public License LGPL 2.1 and BSD license.
 ******************************************************************************/

#ifndef KELOTULIP_ESCDIAGNOSTICS_H
#define KELOTULIP_ESCDIAGNOSTICS_H

extern "C" {
#include "kelo_tulip/soem/ethercattype.h"
#include "nicdrv.h"
#include "kelo_tulip/soem/ethercatbase.h"
#include "kelo_tulip/soem/ethercatmain.h"
}

#include <atomic>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

#include "kelo_tulip/EscCounters.h"

namespace kelo {

struct EscSlaveReading {
	int slave = 0;
	// False when the read got no answer; the counters are then not updated.
	bool ok = false;
	EscCounters counters;
	EscCounterDelta delta;
};

struct EscSnapshot {
	std::uint64_t sequence = 0;
	// Why this read happened: the period elapsed, or a communication error ended.
	bool afterEpisode = false;
	std::vector<EscSlaveReading> slaves;
};

// Reads every slave's ESC error counters, read-only, from a thread of its own
// so that the 1 kHz loop never waits for it. One slave is read per tick, so the
// port is never held for longer than one read. Counters that grew are logged.
class EscDiagnostics {
public:
	EscDiagnostics(ecx_contextt* context, double periodS);
	~EscDiagnostics();

	// Between start() and stop() the port must stay open. Both may be called
	// from any thread, also repeatedly: they serialise on a lock.
	void start();
	void stop();

	// Read once soon, out of turn (any thread, wait-free).
	void requestPoll() { pollRequested_.store(true, std::memory_order_relaxed); }

	std::uint64_t sequence() const { return sequence_.load(std::memory_order_acquire); }
	EscSnapshot snapshot() const;

private:
	void loop();
	void readSlave(int slave, EscSnapshot& round);
	void publish(EscSnapshot&& round);

	ecx_contextt* context_;
	double periodS_;
	std::atomic<bool> pollRequested_{false};
	std::atomic<bool> stopRequested_{false};
	std::atomic<std::uint64_t> sequence_{0};
	std::mutex lifecycle_;
	std::thread thread_;
	EscCounterTracker tracker_;
	mutable std::mutex mutex_;
	EscSnapshot latest_;
};

}  // namespace kelo

#endif  // KELOTULIP_ESCDIAGNOSTICS_H
