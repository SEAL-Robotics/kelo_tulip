/******************************************************************************
 * This software is published under the same dual-license as the rest of
 * kelo_tulip: GNU Lesser General Public License LGPL 2.1 and BSD license.
 ******************************************************************************/

#include "kelo_tulip/ParameterValidation.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>

#include "kelo_tulip/CommandWatchdog.h"

namespace kelo {

namespace {

// NaN fails every comparison, so each check is written to pass only for a
// value inside its range.
bool inOpenClosed(double value, double low, double high) {
	return value > low && value <= high;
}

bool inClosed(double value, double low, double high) {
	return value >= low && value <= high;
}

void check(std::vector<std::string>& errors, bool ok, const std::string& name, double value,
	const std::string& range) {
	if (ok)
		return;
	std::ostringstream message;
	message << name << " = " << value << " is outside " << range;
	errors.push_back(message.str());
}

std::string openClosed(double low, double high) {
	std::ostringstream range;
	range << "(" << low << ", " << high << "]";
	return range.str();
}

std::string closed(double low, double high) {
	std::ostringstream range;
	range << "[" << low << ", " << high << "]";
	return range.str();
}

}  // namespace

double stopRampDurationSec(const DriverLimits& limits) {
	const double linear = limits.vlinMax / std::min(limits.stopVlinDec, limits.vlinDecMax);
	const double angular = limits.vaMax / std::min(limits.stopVaDec, limits.vaDecMax);
	return std::max(linear, angular);
}

std::vector<std::string> limitErrors(const DriverLimits& l) {
	using namespace bounds;
	std::vector<std::string> errors;
	const std::string linVel = openClosed(0, MAX_LINEAR_VELOCITY);
	const std::string angVel = openClosed(0, MAX_ANGULAR_VELOCITY);
	const std::string linAcc = openClosed(0, MAX_LINEAR_ACCELERATION);
	const std::string angAcc = openClosed(0, MAX_ANGULAR_ACCELERATION);
	check(errors, inOpenClosed(l.vlinMax, 0, MAX_LINEAR_VELOCITY), "vlin_max", l.vlinMax, linVel);
	check(errors, inOpenClosed(l.vaMax, 0, MAX_ANGULAR_VELOCITY), "va_max", l.vaMax, angVel);
	check(errors, inOpenClosed(l.vlinAccMax, 0, MAX_LINEAR_ACCELERATION), "vlin_acc_max", l.vlinAccMax, linAcc);
	check(errors, inOpenClosed(l.vlinDecMax, 0, MAX_LINEAR_ACCELERATION), "vlin_dec_max", l.vlinDecMax, linAcc);
	check(errors, inOpenClosed(l.vaAccMax, 0, MAX_ANGULAR_ACCELERATION), "va_acc_max", l.vaAccMax, angAcc);
	check(errors, inOpenClosed(l.vaDecMax, 0, MAX_ANGULAR_ACCELERATION), "va_dec_max", l.vaDecMax, angAcc);
	check(errors, inOpenClosed(l.angleAccMax, 0, MAX_ANGULAR_ACCELERATION), "angle_acc_max", l.angleAccMax, angAcc);
	check(errors, inOpenClosed(l.stopVlinDec, 0, MAX_LINEAR_ACCELERATION), "stop_vlin_dec", l.stopVlinDec, linAcc);
	check(errors, inOpenClosed(l.stopVaDec, 0, MAX_ANGULAR_ACCELERATION), "stop_va_dec", l.stopVaDec, angAcc);
	check(errors, inClosed(l.stopTimeoutSec, MIN_STOP_TIMEOUT, MAX_STOP_TIMEOUT), "stop_timeout",
		l.stopTimeoutSec, closed(MIN_STOP_TIMEOUT, MAX_STOP_TIMEOUT));
	check(errors, CommandWatchdog::isValidTimeout(l.cmdVelTimeoutSec), "cmd_vel_timeout", l.cmdVelTimeoutSec,
		openClosed(0, CommandWatchdog::MAX_TIMEOUT_SEC));

	// Only meaningful once the inputs are; a stop that cannot finish in time
	// would be cut while the base still moves.
	if (errors.empty()) {
		const double needed = stopRampDurationSec(l);
		if (!(needed + STOP_SETTLE_MARGIN <= l.stopTimeoutSec)) {
			std::ostringstream message;
			message << "stop_timeout = " << l.stopTimeoutSec << " s is shorter than the " << needed
				<< " s a stop from vlin_max/va_max takes at stop_vlin_dec/stop_va_dec, plus "
				<< STOP_SETTLE_MARGIN << " s to settle";
			errors.push_back(message.str());
		}
	}
	return errors;
}

std::vector<std::string> wheelModelErrors(const WheelModel& m) {
	using namespace bounds;
	std::vector<std::string> errors;
	const std::string prefix = "wheel model " + m.name + ": ";
	check(errors, inClosed(m.diameter, MIN_WHEEL_DIAMETER, MAX_WHEEL_DIAMETER), prefix + "diameter", m.diameter,
		closed(MIN_WHEEL_DIAMETER, MAX_WHEEL_DIAMETER));
	check(errors, inOpenClosed(m.width, 0, MAX_WHEEL_WIDTH), prefix + "width", m.width,
		openClosed(0, MAX_WHEEL_WIDTH));
	check(errors, inClosed(m.casteroffset, 0, MAX_CASTER_OFFSET), prefix + "casteroffset", m.casteroffset,
		closed(0, MAX_CASTER_OFFSET));
	check(errors, inOpenClosed(m.wheeldistance, 0, MAX_HUB_DISTANCE), prefix + "wheeldistance", m.wheeldistance,
		openClosed(0, MAX_HUB_DISTANCE));
	check(errors, inOpenClosed(m.velocitylimit, 0, MAX_HUB_VELOCITY), prefix + "velocitylimit", m.velocitylimit,
		openClosed(0, MAX_HUB_VELOCITY));
	check(errors, inOpenClosed(m.currentlimit, 0, MAX_TORQUE_LIMIT), prefix + "currentlimit", m.currentlimit,
		openClosed(0, MAX_TORQUE_LIMIT));
	if (std::isfinite(m.currentlimit))
		check(errors, inClosed(m.standbycurrent, 0, m.currentlimit), prefix + "standbycurrent", m.standbycurrent,
			closed(0, m.currentlimit));
	return errors;
}

std::vector<std::string> wheelConfigErrors(const std::vector<WheelConfig>& wheels) {
	using namespace bounds;
	std::vector<std::string> errors;
	const double count = static_cast<double>(wheels.size());
	check(errors, inClosed(count, 1, MAX_WHEELS), "num_wheels", count, closed(1, MAX_WHEELS));

	std::set<int> slaves;
	for (size_t i = 0; i < wheels.size(); i++) {
		const WheelConfig& w = wheels[i];
		const std::string prefix = "wheel" + std::to_string(i) + ".";
		const std::string position = closed(-MAX_WHEEL_POSITION, MAX_WHEEL_POSITION);
		check(errors, inClosed(w.x, -MAX_WHEEL_POSITION, MAX_WHEEL_POSITION), prefix + "x", w.x, position);
		check(errors, inClosed(w.y, -MAX_WHEEL_POSITION, MAX_WHEEL_POSITION), prefix + "y", w.y, position);
		check(errors, std::isfinite(w.a), prefix + "a", w.a, "the finite numbers");
		if (w.ethercatNumber < 1)
			errors.push_back(prefix + "ethercat_number = " + std::to_string(w.ethercatNumber) +
				" is below 1 (EtherCAT slaves count from 1)");
		else if (!slaves.insert(w.ethercatNumber).second)
			errors.push_back(prefix + "ethercat_number = " + std::to_string(w.ethercatNumber) +
				" is used by another wheel");
		for (const std::string& error : wheelModelErrors(w.model))
			errors.push_back("wheel" + std::to_string(i) + ": " + error);
	}
	return errors;
}

} // namespace kelo
