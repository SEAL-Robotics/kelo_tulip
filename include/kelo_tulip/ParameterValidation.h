/******************************************************************************
 * This software is published under the same dual-license as the rest of
 * kelo_tulip: GNU Lesser General Public License LGPL 2.1 and BSD license.
 ******************************************************************************/

#ifndef KELOTULIP_PARAMETERVALIDATION_H
#define KELOTULIP_PARAMETERVALIDATION_H

#include <string>
#include <vector>

#include "kelo_tulip/WheelConfig.h"

namespace kelo {

//! Platform limits and timing as read from the parameters. The driver refuses
//! to start unless every one is valid: a negative velocity limit drives with
//! no command, a zero deceleration never stops.
struct DriverLimits {
	double vlinMax = 1.0;     // m/s
	double vaMax = 1.0;       // rad/s
	double vlinAccMax = 0.5;  // m/s^2
	double vlinDecMax = 0.8;  // m/s^2
	double vaAccMax = 0.5;    // rad/s^2
	double vaDecMax = 0.8;    // rad/s^2
	double angleAccMax = 0.8;
	double stopVlinDec = 0.8;  // m/s^2, ramp on shutdown
	double stopVaDec = 0.8;    // rad/s^2, ramp on shutdown
	double stopTimeoutSec = 3.0;
	double cmdVelTimeoutSec = 0.2;
};

//! Physical bounds. Generous for any KELO platform; a value outside them is a
//! typo or a unit mix-up, not a tuning choice.
namespace bounds {
constexpr double MAX_LINEAR_VELOCITY = 4.0;      // m/s
constexpr double MAX_ANGULAR_VELOCITY = 6.3;     // rad/s
constexpr double MAX_LINEAR_ACCELERATION = 10.0;  // m/s^2
constexpr double MAX_ANGULAR_ACCELERATION = 20.0;  // rad/s^2
constexpr double MIN_STOP_TIMEOUT = 0.5;         // s
// Plus the 1 s backstop in main, this stays below the 10 s after which ros2
// launch SIGKILLs a process that ignores its SIGINT: a kill mid-ramp is a cut.
constexpr double MAX_STOP_TIMEOUT = 5.0;         // s
// Settling and the disable frames, on top of the ramp itself.
constexpr double STOP_SETTLE_MARGIN = 0.5;       // s
constexpr int MAX_WHEELS = 16;
constexpr double MIN_WHEEL_DIAMETER = 0.02;      // m
constexpr double MAX_WHEEL_DIAMETER = 0.5;       // m
constexpr double MAX_WHEEL_WIDTH = 0.3;          // m
constexpr double MAX_CASTER_OFFSET = 0.2;        // m
constexpr double MAX_HUB_DISTANCE = 0.5;         // m
constexpr double MAX_HUB_VELOCITY = 200.0;       // rad/s
constexpr double MAX_MOTOR_CURRENT = 40.0;       // A
constexpr double MAX_WHEEL_POSITION = 10.0;      // m from the platform centre
}  // namespace bounds

//! Time a stop from the velocity limits takes at the shutdown deceleration,
//! which never exceeds the configured one. x, y and yaw ramp independently.
double stopRampDurationSec(const DriverLimits& limits);

//! One message per problem; empty when every limit is valid.
std::vector<std::string> limitErrors(const DriverLimits& limits);
std::vector<std::string> wheelModelErrors(const WheelModel& model);
//! Wheel count, positions, EtherCAT numbers (unique, from 1) and each wheel's
//! model after per-wheel overrides.
std::vector<std::string> wheelConfigErrors(const std::vector<WheelConfig>& wheels);

} // namespace kelo

#endif // KELOTULIP_PARAMETERVALIDATION_H
