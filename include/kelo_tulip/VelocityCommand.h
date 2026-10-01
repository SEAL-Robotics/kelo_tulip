/******************************************************************************
 * This software is published under the same dual-license as the rest of
 * kelo_tulip: GNU Lesser General Public License LGPL 2.1 and BSD license.
 ******************************************************************************/

#ifndef KELOTULIP_VELOCITYCOMMAND_H
#define KELOTULIP_VELOCITYCOMMAND_H

#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>

namespace kelo {

struct VelocityCommand {
	double vx = 0.0;
	double vy = 0.0;
	double va = 0.0;
};

//! A NaN or Inf in any axis makes the whole command a stop: one broken axis
//! means the source is broken, and the ramp would carry NaN to the drives.
//! Returns false when the command had to be replaced.
inline bool sanitizeCommand(VelocityCommand& command) {
	// The controller works in float: a double beyond its range becomes Inf there.
	const auto representable = [](double v) {
		return std::isfinite(v) && std::fabs(v) <= std::numeric_limits<float>::max();
	};
	if (representable(command.vx) && representable(command.vy) && representable(command.va))
		return true;
	command = VelocityCommand();
	return false;
}

struct StampedCommand {
	VelocityCommand command;
	double stampMs = 0.0;   // on the driver's monotonic clock
	std::uint64_t seq = 0;  // 0: nothing commanded yet
};

//! True once the last command is older than the timeout. Nothing commanded
//! yet is not expired: the target is still zero. A non-finite age counts as
//! expired, so a clock anomaly stops the base instead of holding a target.
inline bool commandExpired(const StampedCommand& latest, double nowMs, double timeoutMs) {
	if (latest.seq == 0)
		return false;
	return !(nowMs - latest.stampMs <= timeoutMs);
}

//! Latest-wins hand-over from the ROS thread to the EtherCAT thread. The
//! EtherCAT side never blocks: while the writer holds the lock it keeps the
//! command it fetched one cycle earlier.
class CommandMailbox {
public:
	void post(const VelocityCommand& command, double stampMs) {
		std::lock_guard<std::mutex> lock(mutex_);
		latest_.command = command;
		latest_.stampMs = stampMs;
		latest_.seq++;
	}

	bool tryFetch(StampedCommand& out) {
		std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
		if (!lock.owns_lock())
			return false;
		out = latest_;
		return true;
	}

private:
	std::mutex mutex_;
	StampedCommand latest_;
};

} // namespace kelo

#endif // KELOTULIP_VELOCITYCOMMAND_H
