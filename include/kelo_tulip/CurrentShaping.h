/******************************************************************************
 * This software is published under the same dual-license as the rest of
 * kelo_tulip: GNU Lesser General Public License LGPL 2.1 and BSD license.
 ******************************************************************************/

#ifndef KELOTULIP_CURRENTSHAPING_H
#define KELOTULIP_CURRENTSHAPING_H

#include <algorithm>
#include <cmath>
#include <string>

namespace kelo {

// Peak motor current drives EtherCAT frame loss, so these helpers only ever
// make a hub setpoint slower or smaller than the plain controller would
// command. Every knob at its default is a no-op.
struct CurrentShapingConfig {
	//! Hub setpoint may grow by at most this much per second (rad/s per s).
	//! Only growth in magnitude is limited: shrinking or braking is free, so
	//! the shaping never delays a stop. 0 disables.
	float slewRateRadPerSecSq = 0.0f;
	//! Gain from pivot error (rad) to differential wheel speed (m/s).
	float pivotKp = 0.2f;
	//! Pivot error is clipped to this before applying the gain (rad).
	float maxPivotError = static_cast<float>(M_PI) * 0.25f;
	//! Cap on the differential pivot correction speed (m/s). 0 disables.
	float maxPivotCorrectionSpeed = 0.0f;
	//! Translation is scaled down once the worst caster is this far (rad)
	//! from its target angle. 0 disables re-orient-first.
	float reorientStartError = 0.0f;
	//! Error (rad) at which the translation scale reaches reorientMinScale.
	float reorientFullError = 1.0f;
	//! Translation scale (0..1) while casters are far off target.
	float reorientMinScale = 1.0f;
};

//! The pre-shaping controller's pivot gain and clip; shaping may never exceed
//! the correction these give.
constexpr float LEGACY_PIVOT_KP = 0.2f;
constexpr float LEGACY_MAX_PIVOT_ERROR = static_cast<float>(M_PI * 0.25f);
//! The slew only sees this much of a step; the EtherCAT loop runs at 1 kHz.
constexpr float SLEW_MAX_DT_SEC = 0.003f;
//! A gap this long between control cycles means the driver was paused. Just
//! above the 100 ms the hubs keep driving after their last setpoint
//! (rxdata.timestamp = current_ts + 100 ms in PlatformDriver::doControl): a
//! shorter gap leaves them moving, and resetting then would brake and re-ramp.
constexpr float PAUSE_GAP_SEC = 0.12f;
//! Floor for reorientMinScale while re-orient-first is on: near 0 the base
//! would stall waiting on a caster that has no torque to turn.
constexpr float MIN_REORIENT_SCALE = 0.1f;

//! Rejects values that would make shaping unsafe or undefined. Shaping may
//! only lower setpoints, so the gain and error clip cannot exceed the legacy
//! ones, and a re-orient floor near 0 could stall the base.
inline bool isValid(const CurrentShapingConfig &c, std::string *why = nullptr) {
	auto fail = [why](const char *msg) {
		if (why)
			*why = msg;
		return false;
	};
	if (!std::isfinite(c.slewRateRadPerSecSq) || c.slewRateRadPerSecSq < 0.0f)
		return fail("wheel_slew_rate must be finite and >= 0");
	if (!std::isfinite(c.pivotKp) || c.pivotKp < 0.0f || c.pivotKp > LEGACY_PIVOT_KP)
		return fail("pivot_kp must be finite and in [0, 0.2]");
	if (!std::isfinite(c.maxPivotError) || c.maxPivotError <= 0.0f ||
	    c.maxPivotError > LEGACY_MAX_PIVOT_ERROR)
		return fail("pivot_max_error must be finite and in (0, pi/4]");
	if (!std::isfinite(c.maxPivotCorrectionSpeed) || c.maxPivotCorrectionSpeed < 0.0f)
		return fail("pivot_max_correction_speed must be finite and >= 0");
	if (!std::isfinite(c.reorientStartError) || c.reorientStartError < 0.0f)
		return fail("reorient_start_error must be finite and >= 0");
	if (!std::isfinite(c.reorientFullError) || c.reorientFullError <= c.reorientStartError)
		return fail("reorient_full_error must be finite and > reorient_start_error");
	if (!std::isfinite(c.reorientMinScale) || c.reorientMinScale < 0.0f || c.reorientMinScale > 1.0f)
		return fail("reorient_min_scale must be finite and in [0, 1]");
	if (c.reorientStartError > 0.0f && c.reorientMinScale < MIN_REORIENT_SCALE)
		return fail("reorient_min_scale must be >= 0.1 when re-orient-first is enabled");
	return true;
}

//! One hub's setpoint before clipping. The candidates are the shaped
//! setpoint with and without the translation scale, and the legacy setpoint;
//! the smallest magnitude wins, and a result that would point the other way
//! from the legacy setpoint becomes 0. So |result| <= |legacy| always.
inline float hubSetpoint(float vel, float delta, float legacyDelta, float scale) {
	const float legacy = vel + legacyDelta;
	float best = vel * scale + delta;
	const float plain = vel + delta;
	if (std::fabs(plain) < std::fabs(best))
		best = plain;
	if (std::fabs(legacy) < std::fabs(best))
		best = legacy;
	// Zeroing can weaken the pivot differential by up to ~26% under
	// re-orient-first: slower re-orientation, never a stall.
	return best * legacy < 0.0f ? 0.0f : best;
}

//! The pre-shaping pivot correction for this error.
inline float legacyPivotCorrection(float pivotError) {
	return std::max(-LEGACY_MAX_PIVOT_ERROR, std::min(pivotError, LEGACY_MAX_PIVOT_ERROR)) *
	       LEGACY_PIVOT_KP;
}

//! Moves prev towards target by at most maxRate * dt, but never lets the
//! magnitude exceed the target's: a decrease in magnitude, or a sign flip
//! (which passes through zero), is unrestricted. Braking on a reversal is
//! left unshaped on purpose: it is bounded by currentlimit, and delaying it
//! would make the hub run against the new command.
inline float slewLimit(float prev, float target, float maxRate, float dt) {
	if (maxRate <= 0.0f)
		return target;
	if (prev * target < 0.0f)
		prev = 0.0f;
	if (std::fabs(target) <= std::fabs(prev))
		return target;
	const float maxStep = maxRate * std::max(dt, 0.0f);
	return prev + std::max(-maxStep, std::min(target - prev, maxStep));
}

//! Differential wheel speed that turns a caster towards its target.
inline float pivotCorrectionSpeed(float pivotError, const CurrentShapingConfig &cfg) {
	const float error = std::max(-cfg.maxPivotError, std::min(pivotError, cfg.maxPivotError));
	float speed = error * cfg.pivotKp;
	if (cfg.maxPivotCorrectionSpeed > 0.0f) {
		speed = std::max(-cfg.maxPivotCorrectionSpeed,
		                 std::min(speed, cfg.maxPivotCorrectionSpeed));
	}
	return speed;
}

//! Factor in [reorientMinScale, 1] for the translational command: 1 while the
//! casters are near their target, falling linearly to the minimum at
//! reorientFullError.
inline float reorientScale(float maxAbsPivotError, const CurrentShapingConfig &cfg) {
	if (cfg.reorientStartError <= 0.0f || cfg.reorientMinScale >= 1.0f)
		return 1.0f;
	const float minScale = std::max(cfg.reorientMinScale, 0.0f);
	const float err = std::fabs(maxAbsPivotError);
	if (err <= cfg.reorientStartError)
		return 1.0f;
	if (err >= cfg.reorientFullError)
		return minScale;
	const float t = (err - cfg.reorientStartError) / (cfg.reorientFullError - cfg.reorientStartError);
	return 1.0f - t * (1.0f - minScale);
}

}  // namespace kelo

#endif
