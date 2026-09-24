/******************************************************************************
 * This software is published under the same dual-license as the rest of
 * kelo_tulip: GNU Lesser General Public License LGPL 2.1 and BSD license.
 ******************************************************************************/

#ifndef KELOTULIP_COMMANDWATCHDOG_H
#define KELOTULIP_COMMANDWATCHDOG_H

#include <chrono>
#include <cmath>

namespace kelo {

//! Detects when a command stream has gone silent.
//! Time is passed in by the caller so the logic can be tested without sleeping;
//! use a steady clock so wall-clock jumps cannot fire or mask a timeout.
class CommandWatchdog {
public:
	typedef std::chrono::steady_clock Clock;

	static constexpr double DEFAULT_TIMEOUT_SEC = 0.2;
	//! Upper bound so a mistyped value (e.g. milliseconds) cannot turn into
	//! minutes of travel on a stale command.
	static constexpr double MAX_TIMEOUT_SEC = 2.0;

	explicit CommandWatchdog(double timeoutSec = DEFAULT_TIMEOUT_SEC)
		: timeout(toDuration(DEFAULT_TIMEOUT_SEC))
		, lastCommand()
		, armed(false)
	{
		setTimeout(timeoutSec);
	}

	static bool isValidTimeout(double timeoutSec) {
		return std::isfinite(timeoutSec) && timeoutSec > 0 && timeoutSec <= MAX_TIMEOUT_SEC;
	}

	//! Returns false and keeps the current timeout if timeoutSec is not in
	//! (0, MAX_TIMEOUT_SEC]; the watchdog cannot be disabled.
	bool setTimeout(double timeoutSec) {
		if (!isValidTimeout(timeoutSec))
			return false;

		timeout = toDuration(timeoutSec);
		return true;
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
