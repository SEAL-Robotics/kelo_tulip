/******************************************************************************
 * This software is published under the same dual-license as the rest of
 * kelo_tulip: GNU Lesser General Public License LGPL 2.1 and BSD license.
 ******************************************************************************/

#ifndef KELOTULIP_COMMANDWATCHDOG_H
#define KELOTULIP_COMMANDWATCHDOG_H

#include <chrono>

namespace kelo {

//! Detects when a command stream has gone silent.
//! Time is passed in by the caller so the logic can be tested without sleeping;
//! use a steady clock so wall-clock jumps cannot fire or mask a timeout.
class CommandWatchdog {
public:
	typedef std::chrono::steady_clock Clock;

	explicit CommandWatchdog(double timeoutSec = 0.2)
		: timeout(toDuration(timeoutSec))
		, lastCommand()
		, armed(false)
	{
	}

	void setTimeout(double timeoutSec) {
		timeout = toDuration(timeoutSec);
	}

	double getTimeout() const {
		return std::chrono::duration<double>(timeout).count();
	}

	//! Record that a command arrived at time now.
	void kick(Clock::time_point now) {
		lastCommand = now;
		armed = true;
	}

	//! Returns true once per silence: on the first call after the last command
	//! has become older than the timeout. Returns false before the first command,
	//! since nothing has been commanded that would need stopping.
	bool checkExpired(Clock::time_point now) {
		if (!armed || now - lastCommand <= timeout)
			return false;

		armed = false;
		return true;
	}

private:
	static Clock::duration toDuration(double sec) {
		return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(sec));
	}

	Clock::duration timeout;
	Clock::time_point lastCommand;
	bool armed;
};

} // namespace kelo

#endif // KELOTULIP_COMMANDWATCHDOG_H
