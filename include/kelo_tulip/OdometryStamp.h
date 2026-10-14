/******************************************************************************
 * This software is published under the same dual-license as the rest of
 * kelo_tulip: GNU Lesser General Public License LGPL 2.1 and BSD license.
 ******************************************************************************/

#ifndef KELOTULIP_ODOMETRYSTAMP_H
#define KELOTULIP_ODOMETRYSTAMP_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "kelo_tulip/SampleInstant.h"

namespace kelo {

struct OdomPose {
	double x = 0.0;
	double y = 0.0;
	double a = 0.0;
};

inline double wrapAngle(double a) {
	// The loops would never end on an infinity.
	if (!std::isfinite(a))
		return a;
	while (a < -M_PI)
		a += 2.0 * M_PI;
	while (a > M_PI)
		a -= 2.0 * M_PI;
	return a;
}

// One dead-reckoning step: a constant body twist over dt, moved along the
// chord at the step's mean heading.
inline OdomPose integratePose(const OdomPose& p, double vx, double vy, double va, double dt) {
	const double dx = vx * dt;
	const double dy = vy * dt;
	const double meanHeading = wrapAngle(p.a + 0.5 * va * dt);
	const double s = std::sin(meanHeading);
	const double c = std::cos(meanHeading);
	return OdomPose{p.x + dx * c - dy * s, p.y + dx * s + dy * c, wrapAngle(p.a + va * dt)};
}

// How a step measured its velocity, which decides the instant it describes.
enum class TwistSource {
	EncoderWindow,  // encoder deltas: the mean since the last used sample
	DriveVelocity,  // the drives' own velocities at the newest sample
	Held,           // no new sample from every wheel: the pose did not move
};

inline TwistSource twistSource(bool held, bool everyWheelUsedEncoders) {
	if (held)
		return TwistSource::Held;
	return everyWheelUsedEncoders ? TwistSource::EncoderWindow : TwistSource::DriveVelocity;
}

// The newest instant every wheel has a sample for. Wheels read in one frame
// share its instant; a wheel that missed the last frames holds the window
// back to its own sample, so the window never claims what it did not measure.
inline int64_t commonSampleInstantNs(const std::vector<int64_t>& wheelInstantsNs) {
	if (wheelInstantsNs.empty())
		return kNoInstantNs;
	int64_t common = wheelInstantsNs.front();
	for (const int64_t t : wheelInstantsNs) {
		if (t == kNoInstantNs)
			return kNoInstantNs;
		common = std::min(common, t);
	}
	return common;
}

// Stamps each odometry message at the instant its twist describes, and says
// which pose belongs there. An encoder window's mean velocity is, under
// constant acceleration, the velocity at the window's midpoint: the message is
// stamped there, with the pose integrated over the first half of the window.
// The drives' own velocities belong to the newest sample, as does the pose
// integrated to it. A held step's pose is the one integrated to the last used
// sample, so it repeats that instant. The window's length comes from the
// drives' counters, its end from the host clock, so a clock step moves the
// stamp but never stretches the window.
class OdometryClock {
public:
	struct Stamp {
		int64_t ns;
		double poseFraction;  // of the step's integration that the stamped pose includes
	};

	// windowEndNs: commonSampleInstantNs of the step's samples. Without one
	// the ROS loop's clock stands in for it, a later bound.
	Stamp next(TwistSource source, int64_t windowEndNs, double windowSec, int64_t rosNowNs) {
		if (source == TwistSource::Held)
			return Stamp{lastEndNs_ == kNoInstantNs ? rosNowNs : lastEndNs_, 1.0};
		const int64_t endNs = windowEndNs == kNoInstantNs ? rosNowNs : windowEndNs;
		lastEndNs_ = endNs;
		if (source == TwistSource::DriveVelocity || !(windowSec > 0.0) || !std::isfinite(windowSec))
			return Stamp{endNs, 1.0};
		return Stamp{endNs - static_cast<int64_t>(std::llround(0.5e9 * windowSec)), 0.5};
	}

private:
	int64_t lastEndNs_ = kNoInstantNs;
};

}  // namespace kelo

#endif  // KELOTULIP_ODOMETRYSTAMP_H
