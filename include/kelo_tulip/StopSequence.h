/******************************************************************************
 * This software is published under the same dual-license as the rest of
 * kelo_tulip: GNU Lesser General Public License LGPL 2.1 and BSD license.
 ******************************************************************************/

#ifndef KELOTULIP_STOPSEQUENCE_H
#define KELOTULIP_STOPSEQUENCE_H

#include <cmath>

namespace kelo {

//! The drives have no brakes and coast once they lose their setpoint, so the
//! driver never leaves the bus with a moving base: it ramps the platform to
//! zero, holds zero with the drives enabled until the hubs are still, and only
//! then disables the drives. Every phase is bounded by one deadline.
enum class StopPhase {
	Running,    // no stop requested
	Ramping,    // target forced to zero, setpoints ramping down
	Settling,   // setpoints at zero, drives enabled, hubs coming to rest
	Disabling,  // drives commanded disabled for a few cycles
	Done,       // the EtherCAT loop may end
};

struct StopConfig {
	//! From the request to the start of Disabling, whatever the base does.
	double timeoutMs = 3000.0;
	//! Zero is held at least this long, so the drives' own velocity loop
	//! brakes the hubs before they are released.
	double minSettleMs = 100.0;
	//! Hubs slower than this count as still (rad/s; about 0.04 m/s on a
	//! 165 mm wheel).
	double stillSpeedRadPerSec = 0.5;
	//! ...and must stay so for this long.
	double stillForMs = 50.0;
	//! Disable frames sent before the loop may end.
	double disableMs = 20.0;
};

class StopSequence {
public:
	explicit StopSequence(const StopConfig& config = StopConfig()) : config_(config) {}

	void setConfig(const StopConfig& config) { config_ = config; }
	const StopConfig& config() const { return config_; }

	//! needsRamp is false when the drives never moved the base (not yet
	//! active): there is nothing to ramp, so they are disabled at once.
	void request(double nowMs, bool needsRamp) {
		if (phase_ != StopPhase::Running)
			return;
		startMs_ = nowMs;
		if (needsRamp)
			phase_ = StopPhase::Ramping;
		else
			enterDisabling(nowMs, false);
	}

	bool requested() const { return phase_ != StopPhase::Running; }
	StopPhase phase() const { return phase_; }
	bool drivesEnabled() const { return phase_ == StopPhase::Ramping || phase_ == StopPhase::Settling; }
	//! The deadline cut the ramp or the settling short.
	bool timedOut() const { return timedOut_; }

	//! One control cycle. commandedAtRest: the platform target and every wheel
	//! setpoint have reached zero. maxHubSpeed: the fastest measured hub, rad/s;
	//! a non-finite reading never counts as still.
	StopPhase step(double nowMs, bool commandedAtRest, double maxHubSpeed) {
		const bool overdue = !(nowMs - startMs_ < config_.timeoutMs);
		switch (phase_) {
			case StopPhase::Running:
			case StopPhase::Done:
				break;
			case StopPhase::Ramping:
				if (overdue)
					enterDisabling(nowMs, true);
				else if (commandedAtRest) {
					phase_ = StopPhase::Settling;
					settleStartMs_ = nowMs;
					stillSinceMs_ = NAN;
					updateStill(nowMs, maxHubSpeed);
				}
				break;
			case StopPhase::Settling:
				updateStill(nowMs, maxHubSpeed);
				if (overdue)
					enterDisabling(nowMs, true);
				else if (nowMs - settleStartMs_ >= config_.minSettleMs && std::isfinite(stillSinceMs_)
					&& nowMs - stillSinceMs_ >= config_.stillForMs)
					enterDisabling(nowMs, false);
				break;
			case StopPhase::Disabling:
				if (!(nowMs - disableStartMs_ < config_.disableMs))
					phase_ = StopPhase::Done;
				break;
		}
		return phase_;
	}

private:
	void enterDisabling(double nowMs, bool timedOut) {
		phase_ = StopPhase::Disabling;
		disableStartMs_ = nowMs;
		timedOut_ = timedOut;
	}

	void updateStill(double nowMs, double maxHubSpeed) {
		if (std::isfinite(maxHubSpeed) && std::fabs(maxHubSpeed) < config_.stillSpeedRadPerSec) {
			if (!std::isfinite(stillSinceMs_))
				stillSinceMs_ = nowMs;
		} else {
			stillSinceMs_ = NAN;
		}
	}

	StopConfig config_;
	StopPhase phase_ = StopPhase::Running;
	bool timedOut_ = false;
	double startMs_ = 0.0;
	double settleStartMs_ = 0.0;
	double stillSinceMs_ = NAN;
	double disableStartMs_ = 0.0;
};

} // namespace kelo

#endif // KELOTULIP_STOPSEQUENCE_H
