/******************************************************************************
 * This software is published under the same dual-license as the rest of
 * kelo_tulip: GNU Lesser General Public License LGPL 2.1 and BSD license.
 ******************************************************************************/

#ifndef KELOTULIP_WHEELRECOVERY_H
#define KELOTULIP_WHEELRECOVERY_H

#include <cstdint>

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

}  // namespace kelo

#endif  // KELOTULIP_WHEELRECOVERY_H
