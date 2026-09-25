/******************************************************************************
 * This software is published under the same dual-license as the rest of
 * kelo_tulip: GNU Lesser General Public License LGPL 2.1 and BSD license.
 ******************************************************************************/

#ifndef KELOTULIP_DRIVERLIVENESS_H
#define KELOTULIP_DRIVERLIVENESS_H

namespace kelo {

enum class LoopVerdict {
	Continue,
	EthercatStopped,
	ReinitFailed
};

// The EtherCAT loop can give up (wheels never ready, a module failing) while
// the ROS loop keeps spinning and publishing. Such a driver commands nothing
// yet looks alive, so it has to exit: the launch treats platform_driver's exit
// as fatal and systemd restarts the stack, which re-runs INIT.
inline LoopVerdict checkEthercatLiveness(bool reinitAttempted, bool reinitSucceeded, bool ethercatStopped) {
	if (reinitAttempted && !reinitSucceeded)
		return LoopVerdict::ReinitFailed;
	if (ethercatStopped)
		return LoopVerdict::EthercatStopped;
	return LoopVerdict::Continue;
}

// Distinct codes for someone running platform_driver by hand. Under
// mobile_base.launch.py the on_exit Shutdown makes the launch exit 0 either
// way, which is why mobile-base.service restarts on always, not on-failure.
inline int exitCode(LoopVerdict verdict) {
	switch (verdict) {
		case LoopVerdict::Continue: return 0;
		case LoopVerdict::EthercatStopped: return 2;
		case LoopVerdict::ReinitFailed: return 3;
	}
	return 3;
}

}  // namespace kelo

#endif  // KELOTULIP_DRIVERLIVENESS_H
