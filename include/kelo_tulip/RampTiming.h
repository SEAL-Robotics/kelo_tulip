/******************************************************************************
 * This software is published under the same dual-license as the rest of
 * kelo_tulip: GNU Lesser General Public License LGPL 2.1 and BSD license.
 ******************************************************************************/

#ifndef KELOTULIP_RAMPTIMING_H
#define KELOTULIP_RAMPTIMING_H

#include <algorithm>
#include <cmath>

#include "kelo_tulip/CurrentShaping.h"

namespace kelo {

//! One ramp update never accelerates by more than this much time: a stalled
//! loop or a clock anomaly must not turn into a velocity step.
constexpr float MAX_ACCEL_DT_SEC = 0.01f;
//! Braking may use up to the pause gap, so a loop that runs slow still stops
//! at the configured deceleration rather than slower.
constexpr float MAX_DECEL_DT_SEC = PAUSE_GAP_SEC;

struct RampStep {
	float accelDt;  // s, for growth of a ramped velocity
	float decelDt;  // s, for its decay towards the target
	bool paused;    // the gap was long enough that the hubs are at rest
};

//! A negative or non-finite time step is a clock anomaly, never elapsed time:
//! it ramps by nothing. A negative step used to swap the clip bounds and
//! command the velocity limit to an idle base.
inline RampStep rampStep(float rawDtSec) {
	if (!std::isfinite(rawDtSec) || rawDtSec <= 0.0f)
		return RampStep{0.0f, 0.0f, false};
	return RampStep{std::min(rawDtSec, MAX_ACCEL_DT_SEC), std::min(rawDtSec, MAX_DECEL_DT_SEC),
		rawDtSec > PAUSE_GAP_SEC};
}

} // namespace kelo

#endif // KELOTULIP_RAMPTIMING_H
