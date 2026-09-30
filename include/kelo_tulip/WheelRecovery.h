/******************************************************************************
 * This software is published under the same dual-license as the rest of
 * kelo_tulip: GNU Lesser General Public License LGPL 2.1 and BSD license.
 ******************************************************************************/

#ifndef KELOTULIP_WHEELRECOVERY_H
#define KELOTULIP_WHEELRECOVERY_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace kelo {

// A KELOdrive V2 in normal operation: both motors enabled, all encoders OK.
constexpr std::uint16_t kStatus1Healthy = 63;
constexpr std::uint16_t kStatus2Healthy = 2051;

// Only an enabled wheel can need recovery: recovering a disabled one would
// re-enable a wheel that was switched off on purpose.
inline bool wheelNeedsRecovery(std::uint16_t status1, std::uint16_t status2, bool enabled) {
	return enabled && (status1 != kStatus1Healthy || status2 != kStatus2Healthy);
}

// The enable actually sent to a wheel. Recovery may only withhold it, never
// grant it: an operator's disable wins even mid-recovery.
inline bool effectiveWheelEnable(bool operatorEnable, bool recoveryAllows) {
	return operatorEnable && recoveryAllows;
}

enum class InitAction {
	Wait,
	ResetWheels,
	GiveUp
};

// What INIT does when the wheels are not ready: keep waiting within the
// timeout, then re-run the disable-then-enable start sequence a bounded number
// of times before giving up (which stops EtherCAT and restarts the stack).
inline InitAction initTimeoutAction(bool ready, unsigned int stepsInAttempt, unsigned int timeoutSteps,
	unsigned int resetsDone, unsigned int maxResets) {
	if (ready || stepsInAttempt <= timeoutSteps)
		return InitAction::Wait;
	return resetsDone < maxResets ? InitAction::ResetWheels : InitAction::GiveUp;
}

// Per wheel, as seen by the recovery state machine.
enum WheelState {
	WHEEL_NORMAL_OPERATION  = 1,
	WHEEL_FAILURE = 2, //permanent failure
	WHEEL_STATUS_RECOVERY_SENDING_DISABLE = 3,
	WHEEL_STATUS_RECOVERY_SENDING_ENABLE  = 4,
	WHEEL_STATUS_RECOVERY_WAITIING_FOR_NORMAL_OPERATION = 5,
	// The slave is not answering: the wheel is held disabled, no attempt is spent.
	WHEEL_LINK_LOST = 6
};

// status1 as a KELOdrive V2 reports it. Seen: 63 in normal operation; 61
// (ENABLED2 clear) on a wheel whose second motor does not enable and on a slave
// back from a dropout, where 4157 (0x103D) shows for a moment first; 125 (61 with
// 0x40 set) on a wheel whose supply collapsed. 0x20, named UNDERVOLTAGE, is set in
// normal operation and 0x40, named OVERVOLTAGE, in that power loss, so the names in
// KeloDriveAPI.h look wrong for this firmware. Pending confirmation from KELO the
// bits are used by value.
constexpr std::uint16_t kStatus1FaultMask = 0x40 | 0x80 | 0x100 | 0x200 | 0x400;
constexpr std::uint16_t kStatus1EncodersOk = 0x04 | 0x08 | 0x10;

// A drive that can be re-enabled: encoders reporting and none of the bits that
// stand for a supply, current or temperature fault. Grip bits (0x800..0x4000)
// are meaningless on a wheel and ignored.
inline bool wheelStatusSane(std::uint16_t status1) {
	return (status1 & kStatus1FaultMask) == 0 && (status1 & kStatus1EncodersOk) == kStatus1EncodersOk;
}

// Whether a wheel's slave is reachable. SOEM's periodic state read reports state
// 0 for every slave after a single lost frame, and frames are lost all the time
// here, so a non-operational state only counts once it has lasted; a slave the
// check thread has declared lost counts at once.
class LinkMonitor {
public:
	explicit LinkMonitor(double debounceMs = 250.0) : debounceMs_(debounceMs) {}

	bool update(double nowMs, bool lost, bool operational) {
		if (operational)
			notOperationalSinceMs_ = kNever;
		else if (notOperationalSinceMs_ == kNever)
			notOperationalSinceMs_ = nowMs;
		const bool lasting = notOperationalSinceMs_ != kNever && nowMs - notOperationalSinceMs_ > debounceMs_;
		return !lost && !lasting;
	}

private:
	static constexpr double kNever = -1.0;
	double debounceMs_;
	double notOperationalSinceMs_ = kNever;
};

struct WheelRecoveryConfig {
	unsigned int maxAttempts = 10;
	double latchMs = 10.0;
	double disableMs = 10.0;
	double reenableMs = 1.0;
	double retryMs = 100.0;
	double counterResetMs = 30000.0;
	// How long a slave may stay unreachable before the wheel is given up.
	double linkLossWindowMs = 5000.0;
	// A returned slave must look sane this long before it is re-enabled.
	double linkStableMs = 100.0;
};

enum RecoveryEvent : unsigned int {
	RECOVERY_EVENT_LINK_LOST = 1u << 0,
	RECOVERY_EVENT_LINK_RESTORED = 1u << 1,
	RECOVERY_EVENT_STARTED = 1u << 2,
	RECOVERY_EVENT_SUCCEEDED = 1u << 3,
	RECOVERY_EVENT_ATTEMPT_FAILED = 1u << 4,
	RECOVERY_EVENT_ATTEMPTS_RESET = 1u << 5,
	RECOVERY_EVENT_GAVE_UP = 1u << 6,
	RECOVERY_EVENT_LINK_LOSS_TIMEOUT = 1u << 7,
	RECOVERY_EVENT_ABORTED_BY_OPERATOR = 1u << 8
};

// One wheel's recovery, with time passed in so it can be tested. A fault on a
// reachable drive spends one of maxAttempts per disable-enable try and gives up
// after them. An unreachable slave spends none, because the retries would run
// out before it can answer: it is held disabled for linkLossWindowMs, then
// re-enabled once it looks sane, and that re-enable is one counted attempt, so
// repeated dropouts still add up to a give-up.
class WheelRecoveryMachine {
public:
	struct Input {
		double nowMs;
		bool operatorEnable;
		bool needsRecovery;
		bool linkUp;
		bool statusSane;
	};
	struct Output {
		bool allowsEnable;
		// The wheel is not in normal operation: the platform must not drive on.
		bool holdAtZero;
		unsigned int events;
	};

	explicit WheelRecoveryMachine(const WheelRecoveryConfig& config = WheelRecoveryConfig(), double nowMs = 0.0)
		: config_(config), entryMs_(nowMs), lastNormalMs_(nowMs), lastAttemptMs_(nowMs) {}

	Output step(const Input& in) {
		unsigned int events = 0;
		const double now = in.nowMs;

		if (isRecovering() && !in.operatorEnable) {
			enter(WHEEL_NORMAL_OPERATION, now);
			events |= RECOVERY_EVENT_ABORTED_BY_OPERATOR;
		}
		if (in.operatorEnable && !in.linkUp && state_ != WHEEL_LINK_LOST && state_ != WHEEL_FAILURE) {
			enter(WHEEL_LINK_LOST, now);
			lossStartMs_ = now;
			linkUpSinceMs_ = kNever;
			events |= RECOVERY_EVENT_LINK_LOST;
		}

		switch (state_) {
			case WHEEL_NORMAL_OPERATION:
				stepNormal(in, events);
				break;
			case WHEEL_FAILURE:
				break;
			case WHEEL_LINK_LOST:
				stepLinkLost(in, events);
				break;
			case WHEEL_STATUS_RECOVERY_SENDING_DISABLE:
				if (now - entryMs_ > config_.disableMs)
					enter(WHEEL_STATUS_RECOVERY_SENDING_ENABLE, now);
				break;
			case WHEEL_STATUS_RECOVERY_SENDING_ENABLE:
				if (now - entryMs_ > config_.reenableMs)
					enter(WHEEL_STATUS_RECOVERY_WAITIING_FOR_NORMAL_OPERATION, now);
				break;
			case WHEEL_STATUS_RECOVERY_WAITIING_FOR_NORMAL_OPERATION:
				stepWaiting(in, events);
				break;
		}

		// The enable goes out on the first cycle of ENABLE, so a fresh
		// transition into it already allows it.
		const bool withheld = state_ == WHEEL_FAILURE || state_ == WHEEL_LINK_LOST ||
			state_ == WHEEL_STATUS_RECOVERY_SENDING_DISABLE;
		return Output{!withheld, state_ != WHEEL_NORMAL_OPERATION, events};
	}

	WheelState state() const { return state_; }
	unsigned int attempts() const { return attempts_; }
	bool failed() const { return state_ == WHEEL_FAILURE; }

private:
	static constexpr double kNever = -1.0;

	bool isRecovering() const {
		return state_ == WHEEL_STATUS_RECOVERY_SENDING_DISABLE ||
			state_ == WHEEL_STATUS_RECOVERY_SENDING_ENABLE ||
			state_ == WHEEL_STATUS_RECOVERY_WAITIING_FOR_NORMAL_OPERATION ||
			state_ == WHEEL_LINK_LOST;
	}

	void enter(WheelState state, double now) {
		state_ = state;
		entryMs_ = now;
	}

	// One counted try: disable, enable, wait; past maxAttempts the wheel is given up.
	void beginAttempt(double now, unsigned int& events) {
		attempts_++;
		if (attempts_ <= config_.maxAttempts) {
			lastAttemptMs_ = now;
			enter(WHEEL_STATUS_RECOVERY_SENDING_DISABLE, now);
			events |= RECOVERY_EVENT_STARTED;
		} else {
			enter(WHEEL_FAILURE, now);
			events |= RECOVERY_EVENT_GAVE_UP;
		}
	}

	void stepNormal(const Input& in, unsigned int& events) {
		if (in.needsRecovery && in.operatorEnable) {
			if (in.nowMs - lastNormalMs_ > config_.latchMs)
				beginAttempt(in.nowMs, events);
		} else {
			lastNormalMs_ = in.nowMs;
		}
		// Error escalation: only a long stretch without a retry forgives the old ones.
		if (state_ == WHEEL_NORMAL_OPERATION && attempts_ > 0 &&
			in.nowMs - lastAttemptMs_ > config_.counterResetMs) {
			attempts_ = 0;
			events |= RECOVERY_EVENT_ATTEMPTS_RESET;
		}
	}

	void stepLinkLost(const Input& in, unsigned int& events) {
		const double now = in.nowMs;
		// Only a slave that answers and looks sane counts as back; one with
		// fault bits stays held and disabled until the window runs out.
		if (!in.linkUp || !in.statusSane)
			linkUpSinceMs_ = kNever;
		else if (linkUpSinceMs_ == kNever)
			linkUpSinceMs_ = now;

		if (linkUpSinceMs_ != kNever && now - linkUpSinceMs_ >= config_.linkStableMs) {
			events |= RECOVERY_EVENT_LINK_RESTORED;
			if (in.needsRecovery) {
				beginAttempt(now, events);
			} else if (++attempts_ > config_.maxAttempts) {
				// Even a clean return costs one, so repeated dropouts add up.
				enter(WHEEL_FAILURE, now);
				events |= RECOVERY_EVENT_GAVE_UP;
			} else {
				lastAttemptMs_ = now;
				lastNormalMs_ = now;
				enter(WHEEL_NORMAL_OPERATION, now);
			}
			return;
		}
		// A slave that is back and sane finishes its stability check first.
		const bool settling = linkUpSinceMs_ != kNever;
		if (!settling && now - lossStartMs_ > config_.linkLossWindowMs) {
			enter(WHEEL_FAILURE, now);
			events |= RECOVERY_EVENT_LINK_LOSS_TIMEOUT | RECOVERY_EVENT_GAVE_UP;
		}
	}

	void stepWaiting(const Input& in, unsigned int& events) {
		if (!in.needsRecovery) {
			events |= RECOVERY_EVENT_SUCCEEDED;
			enter(WHEEL_NORMAL_OPERATION, in.nowMs);
		} else if (in.nowMs - entryMs_ > config_.retryMs) {
			events |= RECOVERY_EVENT_ATTEMPT_FAILED;
			enter(WHEEL_NORMAL_OPERATION, in.nowMs);
		}
	}

	WheelRecoveryConfig config_;
	WheelState state_ = WHEEL_NORMAL_OPERATION;
	unsigned int attempts_ = 0;
	double entryMs_;
	double lastNormalMs_;
	double lastAttemptMs_;
	double lossStartMs_ = 0.0;
	double linkUpSinceMs_ = kNever;
};


// The wheel setpoints as one vector that is ramped, never cut, to zero while
// the platform holds and ramped back afterwards. The whole vector is scaled by
// one factor, so every setpoint reaches its goal in the same cycle and the
// wheels keep a consistent kinematic relation on the way.
class SetpointVectorSlew {
public:
	explicit SetpointVectorSlew(std::size_t size = 0) : last_(size, 0.0f) {}
	void resize(std::size_t size) { last_.assign(size, 0.0f); }
	void reset() { std::fill(last_.begin(), last_.end(), 0.0f); ramping_ = false; }

	// out may alias target. Sizes must equal the constructed size.
	void step(const float* target, bool hold, float downStep, float upStep, float* out) {
		const std::size_t n = last_.size();
		float maxDelta = 0.0f;
		for (std::size_t i = 0; i < n; i++)
			maxDelta = std::max(maxDelta, std::fabs(goalOf(target[i], hold) - last_[i]));
		ramping_ = hold || (ramping_ && maxDelta > 0.0f);

		const float allowed = hold ? downStep : upStep;
		// One factor for the whole vector; the tolerance absorbs float rounding
		// so the last cycle lands exactly on the goal.
		const bool arrives = !ramping_ || maxDelta <= allowed * 1.0001f;
		const float factor = arrives ? 1.0f : allowed / maxDelta;
		for (std::size_t i = 0; i < n; i++) {
			const float goal = goalOf(target[i], hold);
			last_[i] = arrives ? goal : last_[i] + (goal - last_[i]) * factor;
			out[i] = last_[i];
		}
		if (!hold && arrives)
			ramping_ = false;
	}

	const std::vector<float>& last() const { return last_; }

private:
	// A NaN or infinite target means stop, and is never kept.
	static float goalOf(float target, bool hold) { return hold || !std::isfinite(target) ? 0.0f : target; }

	std::vector<float> last_;
	bool ramping_ = false;
};

// Ramp-up per cycle after a hold: the current shaping's slew rate when it is
// set and gentler than the default.
inline float releaseStepPerCycle(float shapingRateRadPerSecSq, float defaultStep) {
	constexpr float kCycleSec = 0.001f;
	if (!(shapingRateRadPerSecSq > 0.0f))
		return defaultStep;
	return std::min(defaultStep, shapingRateRadPerSecSq * kCycleSec);
}

}  // namespace kelo

#endif  // KELOTULIP_WHEELRECOVERY_H
