#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <vector>

#include "kelo_tulip/ParameterValidation.h"

using kelo::DriverLimits;
using kelo::WheelConfig;
using kelo::WheelModel;

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

bool mentions(const std::vector<std::string>& errors, const std::string& needle) {
	for (const std::string& e : errors)
		if (e.find(needle) != std::string::npos)
			return true;
	return false;
}

WheelModel kd165() {
	WheelModel m;
	m.name = "KD165";
	m.diameter = 0.165;
	m.width = 0.06;
	m.casteroffset = 0.015;
	m.wheeldistance = 0.184;
	m.velocitylimit = 100.0;
	m.currentlimit = 0.65;
	m.standbycurrent = 0.1;
	return m;
}

std::vector<WheelConfig> fourWheels() {
	std::vector<WheelConfig> wheels(4);
	const double xs[4] = {1.0325, 1.0325, -1.0325, -1.0325};
	const double ys[4] = {-0.2425, 0.2425, 0.2425, -0.2425};
	for (int i = 0; i < 4; i++) {
		wheels[i].ethercatNumber = i + 1;
		wheels[i].x = xs[i];
		wheels[i].y = ys[i];
		wheels[i].a = 1.5708;
		wheels[i].model = kd165();
	}
	return wheels;
}

// A production-like configuration: 2.2 m/s, braking at 2.5 m/s^2 on exit.
DriverLimits fastPlatformLimits() {
	DriverLimits l;
	l.vlinMax = 2.2;
	l.vaMax = 0.66;
	l.vlinAccMax = 1.1;
	l.vlinDecMax = 2.75;
	l.vaAccMax = 0.9;
	l.vaDecMax = 2.2;
	l.stopVlinDec = 2.5;
	l.stopVaDec = 2.0;
	l.stopTimeoutSec = 3.0;
	l.cmdVelTimeoutSec = 0.2;
	return l;
}

}  // namespace

TEST(LimitErrors, theDefaultsAndAProductionConfigAreValid) {
	EXPECT_TRUE(kelo::limitErrors(DriverLimits()).empty());
	EXPECT_TRUE(kelo::limitErrors(fastPlatformLimits()).empty());
}

class EachLimit : public ::testing::TestWithParam<std::pair<const char*, double DriverLimits::*>> {};

TEST_P(EachLimit, isRejectedWhenNonFiniteZeroNegativeOrAbsurd) {
	const char* name = GetParam().first;
	double DriverLimits::* field = GetParam().second;
	for (const double bad : {kNaN, kInf, -kInf, 0.0, -1.0, 1.0e6}) {
		DriverLimits l = fastPlatformLimits();
		l.*field = bad;
		const std::vector<std::string> errors = kelo::limitErrors(l);
		EXPECT_TRUE(mentions(errors, name)) << name << " = " << bad;
	}
}

INSTANTIATE_TEST_SUITE_P(Limits, EachLimit, ::testing::Values(
	std::make_pair("vlin_max", &DriverLimits::vlinMax),
	std::make_pair("va_max", &DriverLimits::vaMax),
	std::make_pair("vlin_acc_max", &DriverLimits::vlinAccMax),
	std::make_pair("vlin_dec_max", &DriverLimits::vlinDecMax),
	std::make_pair("va_acc_max", &DriverLimits::vaAccMax),
	std::make_pair("va_dec_max", &DriverLimits::vaDecMax),
	std::make_pair("angle_acc_max", &DriverLimits::angleAccMax),
	std::make_pair("stop_vlin_dec", &DriverLimits::stopVlinDec),
	std::make_pair("stop_va_dec", &DriverLimits::stopVaDec),
	std::make_pair("stop_timeout", &DriverLimits::stopTimeoutSec),
	std::make_pair("cmd_vel_timeout", &DriverLimits::cmdVelTimeoutSec)));

TEST(LimitErrors, aStopTimeoutTooShortForTheRampIsRejected) {
	DriverLimits l = fastPlatformLimits();
	l.stopVlinDec = 0.5;  // 2.2 m/s takes 4.4 s
	EXPECT_TRUE(mentions(kelo::limitErrors(l), "shorter than"));
}

TEST(LimitErrors, aStopTimeoutWithNoRoomToSettleIsRejected) {
	DriverLimits l = fastPlatformLimits();
	l.stopVlinDec = 0.8;  // 2.75 s of ramp leaves 0.25 s of a 3 s timeout
	EXPECT_TRUE(mentions(kelo::limitErrors(l), "to settle"));
	l.stopTimeoutSec = 3.25;
	EXPECT_TRUE(kelo::limitErrors(l).empty());
}

TEST(LimitErrors, aStopTimeoutBeyondWhatLaunchWaitsForIsRejected) {
	DriverLimits l = fastPlatformLimits();
	l.stopTimeoutSec = 6.0;
	EXPECT_TRUE(mentions(kelo::limitErrors(l), "stop_timeout"));
}

TEST(LimitErrors, theStopRampNeverBrakesHarderThanTheConfiguredDeceleration) {
	DriverLimits l = fastPlatformLimits();
	l.stopVlinDec = 9.0;
	l.vlinDecMax = 1.0;
	EXPECT_DOUBLE_EQ(kelo::stopRampDurationSec(l), 2.2);
}

TEST(WheelModelErrors, aRealModelIsValid) {
	EXPECT_TRUE(kelo::wheelModelErrors(kd165()).empty());
	EXPECT_TRUE(kelo::wheelModelErrors(WheelModel()).empty());
}

TEST(WheelModelErrors, eachFieldIsChecked) {
	const std::pair<const char*, double WheelModel::*> fields[] = {
		{"diameter", &WheelModel::diameter}, {"width", &WheelModel::width},
		{"casteroffset", &WheelModel::casteroffset}, {"wheeldistance", &WheelModel::wheeldistance},
		{"velocitylimit", &WheelModel::velocitylimit}, {"currentlimit", &WheelModel::currentlimit},
		{"standbycurrent", &WheelModel::standbycurrent},
	};
	for (const auto& field : fields) {
		for (const double bad : {kNaN, kInf, -1.0, 1.0e6}) {
			WheelModel m = kd165();
			m.*(field.second) = bad;
			EXPECT_TRUE(mentions(kelo::wheelModelErrors(m), field.first)) << field.first << " = " << bad;
		}
	}
}

TEST(WheelModelErrors, standbyCurrentAboveTheCurrentLimitIsRejected) {
	WheelModel m = kd165();
	m.standbycurrent = 1.0;
	EXPECT_TRUE(mentions(kelo::wheelModelErrors(m), "standbycurrent"));
}

TEST(WheelConfigErrors, aFourWheelPlatformIsValid) {
	EXPECT_TRUE(kelo::wheelConfigErrors(fourWheels()).empty());
}

TEST(WheelConfigErrors, noWheelsIsRejected) {
	EXPECT_TRUE(mentions(kelo::wheelConfigErrors({}), "num_wheels"));
}

TEST(WheelConfigErrors, aMissingOrDuplicateEtherCATNumberIsRejected) {
	std::vector<WheelConfig> wheels = fourWheels();
	wheels[2].ethercatNumber = 0;
	EXPECT_TRUE(mentions(kelo::wheelConfigErrors(wheels), "wheel2.ethercat_number"));
	wheels = fourWheels();
	wheels[3].ethercatNumber = 1;
	EXPECT_TRUE(mentions(kelo::wheelConfigErrors(wheels), "used by another wheel"));
}

TEST(WheelConfigErrors, aNonFiniteOrFarAwayPositionIsRejected) {
	std::vector<WheelConfig> wheels = fourWheels();
	wheels[1].x = kNaN;
	wheels[2].y = 50.0;
	wheels[3].a = kInf;
	const std::vector<std::string> errors = kelo::wheelConfigErrors(wheels);
	EXPECT_TRUE(mentions(errors, "wheel1.x"));
	EXPECT_TRUE(mentions(errors, "wheel2.y"));
	EXPECT_TRUE(mentions(errors, "wheel3.a"));
}

TEST(WheelConfigErrors, aPerWheelModelOverrideIsChecked) {
	std::vector<WheelConfig> wheels = fourWheels();
	wheels[0].model.diameter = 0.0;
	EXPECT_TRUE(mentions(kelo::wheelConfigErrors(wheels), "wheel0: wheel model KD165: diameter"));
}
