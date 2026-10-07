/******************************************************************************
 * This software is published under the same dual-license as the rest of
 * kelo_tulip: GNU Lesser General Public License LGPL 2.1 and BSD license.
 ******************************************************************************/

#ifndef KELOTULIP_ODOMETRYFRESHNESS_H
#define KELOTULIP_ODOMETRYFRESHNESS_H

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace kelo {

// The EtherCAT thread stops refreshing the process data when the link drops,
// so the wheel data keeps its last values: a stale sensor timestamp, the last
// encoders and the last drive velocities. The wheels sample at 1 kHz and the
// ROS loop runs at 20 Hz, so a timestamp that did not move between two ROS
// steps means no sample arrived. A loop overrun can read the same sample twice
// though, so a wheel is only stale after staleTicks unchanged steps in a row.
// A timestamp of zero means no sample ever arrived and is stale at once. A
// timestamp that moves backwards (drive reset) is still a new sample.
class OdometryFreshnessTracker {
public:
	static constexpr int DEFAULT_STALE_TICKS = 3;
	//! 5 s at the 20 Hz ROS loop.
	static constexpr int DEFAULT_NO_DATA_WARN_TICKS = 100;

	struct Update {
		bool stale;        // any wheel is stale: odometry must read zero
		bool becameStale;  // fresh -> stale, only once data has been seen
		bool becameFresh;  // stale -> fresh, only once data has been seen
		bool noNewSample;  // some wheel repeated its timestamp this step: the
		                   // step carries no information, even if not stale yet
		bool noDataWarning;  // true once, if no wheel data arrived after start
		bool resync;       // stale -> fresh, including the first data: the
		                   // encoder deltas across the gap must not be used
	};

	OdometryFreshnessTracker(size_t nWheels, int staleTicks, int noDataWarnTicks = DEFAULT_NO_DATA_WARN_TICKS)
		: staleTicks(staleTicks < 1 ? 1 : staleTicks)
		, noDataWarnTicks(noDataWarnTicks)
		, ticks(0)
		, prevTs(nWheels, 0)
		, unchanged(nWheels, 0)
		, baselineTs(nWheels, 0)
		, delta(nWheels, 0)
		, stale(nWheels > 0 ? true : false)
		, armed(false)
	{
	}

	Update update(const std::vector<uint64_t>& ts) {
		bool anyStale = false;
		bool anyUnchanged = false;
		for (size_t i = 0; i < prevTs.size(); i++) {
			delta[i] = ts[i] - baselineTs[i];
			unchanged[i] = (ts[i] != prevTs[i]) ? 0 : unchanged[i] + 1;
			prevTs[i] = ts[i];
			if (unchanged[i] > 0)
				anyUnchanged = true;
			if (ts[i] == 0 || unchanged[i] >= staleTicks)
				anyStale = true;
		}
		Update result;
		result.stale = anyStale;
		result.noNewSample = anyUnchanged;
		result.noDataWarning = !armed && !prevTs.empty() && ++ticks == noDataWarnTicks;
		result.becameStale = armed && !stale && anyStale;
		result.becameFresh = armed && stale && !anyStale;
		result.resync = stale && !anyStale;
		if (!anyStale)
			armed = true;
		// The integration baseline only advances on steps that are used, so
		// the next used step's delta spans everything the pose skipped.
		if (!anyStale && !anyUnchanged)
			baselineTs = ts;
		stale = anyStale;
		return result;
	}

	//! Timestamp advance of wheel i since the last step that was not held, in
	//! ns (unsigned, so a timestamp that moved backwards wraps to a huge
	//! value).
	uint64_t deltaNs(size_t i) const {
		return delta[i];
	}

private:
	int staleTicks;
	int noDataWarnTicks;
	int ticks;
	std::vector<uint64_t> prevTs;
	std::vector<int> unchanged;
	std::vector<uint64_t> baselineTs;
	std::vector<uint64_t> delta;
	bool stale;
	bool armed;
};

// A step without a new sample from every wheel must not move the pose: the
// drive velocities in it are the last ones, not a measurement.
inline bool holdOdometry(bool stale, bool noNewSample) {
	return stale || noNewSample;
}

// norm() wraps an encoder difference to +-pi, so a wheel turning at
// maxGroundSpeed / wheelRadius is only unambiguous over gaps below pi / omega.
// 0.8 keeps a margin. A non-positive speed leaves the gap unbounded.
// The speed assumes the guard's limits (vlin_max plus va_max times the wheel
// distance). The controller clips x and y independently, so it can reach
// sqrt(2) * vlin_max, and a pushed or rolling base is not bounded at all; the
// 0.8 factor is the headroom for that, not a guarantee.
inline double encoderAliasingLimitSec(double wheelRadius, double maxGroundSpeed) {
	if (!(maxGroundSpeed > 0.0) || !(wheelRadius > 0.0))
		return std::numeric_limits<double>::max();
	return 0.8 * M_PI * wheelRadius / maxGroundSpeed;
}

// Longest gap over which encoder deltas are still a velocity measurement: the
// gap after the most staleTicks - 1 held steps plus half a period of jitter,
// capped where a fast wheel would alias. Anything longer is measured with the
// drive velocities instead.
inline double encoderDeltaLimitSec(int staleTicks, double loopPeriodSec, double aliasingLimitSec) {
	double debounceLimit = staleTicks * loopPeriodSec + 0.5 * loopPeriodSec;
	return debounceLimit < aliasingLimitSec ? debounceLimit : aliasingLimitSec;
}

// Time the drive velocities are integrated over when the encoder deltas are
// not used. A short gap without resync is motion the pose skipped and is
// recovered. A resync follows an outage, and a gap beyond what the debounce
// can produce (a drive re-init, a clock step, a wrapped timestamp) is not
// motion the pose skipped: neither is integrated beyond one period.
inline double fallbackDtSec(double gapSec, bool resync, double maxGapSec, double loopPeriodSec) {
	if (resync || !(gapSec > 0.0) || gapSec > maxGapSec)
		return loopPeriodSec;
	return gapSec;
}

inline bool isEncoderDeltaUsable(double deltaSec, bool resync, double limitSec) {
	return !resync && deltaSec > 0.0 && deltaSec <= limitSec;
}

inline bool isValidStaleTwistCovariance(double value) {
	return std::isfinite(value) && value > 0;
}

// Same rule for the yaw and yaw-rate variances the odometry reports while the
// wheel data is fresh.
inline bool isValidOdomVariance(double value) {
	return std::isfinite(value) && value > 0;
}

// Zero velocity with a tight covariance tells the estimator the base is
// definitely standing still; while the wheel data is stale that is unknown.
inline double twistCovariance(bool stale, double normal, double staleValue) {
	return stale ? staleValue : normal;
}

}  // namespace kelo

#endif  // KELOTULIP_ODOMETRYFRESHNESS_H
