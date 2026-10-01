#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <limits>
#include <thread>
#include <vector>

#include "kelo_tulip/PlatformDriverROS.h"
#include "rclcpp/rclcpp.hpp"

namespace {

struct Velocity {
	double vx, vy, va;
};

// Records velocity targets instead of handing them to EtherCAT.
class RecordingPlatformDriver : public kelo::PlatformDriver {
public:
	RecordingPlatformDriver(const std::vector<kelo::WheelConfig>& configs, const std::vector<kelo::WheelData>& data)
		: kelo::PlatformDriver(configs, data)
	{
	}

	void setTargetVelocity(double vx, double vy, double va) override {
		targets.push_back(Velocity{vx, vy, va});
	}

	std::vector<Velocity> targets;
};

class TestablePlatformDriverROS : public kelo::PlatformDriverROS {
public:
	RecordingPlatformDriver* recorder() {
		return static_cast<RecordingPlatformDriver*>(driver);
	}

	using kelo::PlatformDriverROS::cmdVelCallback;

	double cmdVelTimeout() const {
		return cmdVelWatchdog.getTimeout();
	}

	rclcpp::QoS cmdVelQos() {
		return cmdVelSubscriber->get_actual_qos();
	}

	// A zeroed process image for the one wheel, so step() has data to read.
	void attachFakeBus() {
		std::memset(slaves, 0, sizeof(slaves));
		std::memset(&inputs, 0, sizeof(inputs));
		std::memset(&outputs, 0, sizeof(outputs));
		slaves[1].inputs = reinterpret_cast<uint8*>(&inputs);
		slaves[1].outputs = reinterpret_cast<uint8*>(&outputs);
		ASSERT_TRUE(recorder()->initEtherCAT(slaves, 1));
	}

protected:
	kelo::PlatformDriver* createDriver() override {
		return new RecordingPlatformDriver(wheelConfigs, wheelData);
	}

	ec_slavet slaves[2];
	txpdo1_t inputs;
	rxpdo1_t outputs;
};

// One wheel with a listed model: the least the driver accepts.
std::vector<rclcpp::Parameter> oneWheel() {
	return {
		rclcpp::Parameter("num_wheels", 1),
		rclcpp::Parameter("wheel_models.list", std::vector<std::string>{"KD165"}),
		rclcpp::Parameter("wheel0.model", "KD165"),
		rclcpp::Parameter("wheel0.ethercat_number", 1),
		rclcpp::Parameter("wheel0.x", 0.5),
	};
}

std::vector<rclcpp::Parameter> with(std::vector<rclcpp::Parameter> params, const rclcpp::Parameter& extra) {
	for (auto& p : params) {
		if (p.get_name() == extra.get_name()) {
			p = extra;
			return params;
		}
	}
	params.push_back(extra);
	return params;
}

geometry_msgs::msg::Twist::SharedPtr twist(double vx, double vy, double va) {
	auto msg = std::make_shared<geometry_msgs::msg::Twist>();
	msg->linear.x = vx;
	msg->linear.y = vy;
	msg->angular.z = va;
	return msg;
}

class PlatformDriverROSWatchdog : public ::testing::Test {
protected:
	static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
	static void TearDownTestSuite() { rclcpp::shutdown(); }

	void SetUp() override {
		rclcpp::NodeOptions options;
		options.parameter_overrides(with(oneWheel(), rclcpp::Parameter("cmd_vel_timeout", 0.05)));
		node = std::make_shared<rclcpp::Node>("platform_driver_watchdog_test", options);
		ASSERT_TRUE(driverRos.init(node, ""));
		driverRos.attachFakeBus();
	}

	rclcpp::Node::SharedPtr node;
	TestablePlatformDriverROS driverRos;
};

TEST_F(PlatformDriverROSWatchdog, readsTimeoutParameter) {
	EXPECT_DOUBLE_EQ(driverRos.cmdVelTimeout(), 0.05);
}

TEST_F(PlatformDriverROSWatchdog, cmdVelKeepsOnlyLatestMessage) {
	// A queued backlog would replay old commands after a stall.
	EXPECT_EQ(driverRos.cmdVelQos().depth(), 1u);
}

TEST_F(PlatformDriverROSWatchdog, freshCommandIsPassedThrough) {
	driverRos.cmdVelCallback(twist(0.1, 0.0, 0.2));
	driverRos.step();

	auto& targets = driverRos.recorder()->targets;
	ASSERT_EQ(targets.size(), 1u);
	EXPECT_DOUBLE_EQ(targets[0].vx, 0.1);
	EXPECT_DOUBLE_EQ(targets[0].va, 0.2);
}

TEST_F(PlatformDriverROSWatchdog, staleCommandIsZeroedOnce) {
	driverRos.cmdVelCallback(twist(0.1, 0.05, 0.2));
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	driverRos.step();
	driverRos.step();

	auto& targets = driverRos.recorder()->targets;
	ASSERT_EQ(targets.size(), 2u);
	EXPECT_DOUBLE_EQ(targets[1].vx, 0.0);
	EXPECT_DOUBLE_EQ(targets[1].vy, 0.0);
	EXPECT_DOUBLE_EQ(targets[1].va, 0.0);
}

TEST_F(PlatformDriverROSWatchdog, aNonFiniteCommandReachesTheDriverAsAStop) {
	driverRos.cmdVelCallback(twist(0.1, std::numeric_limits<double>::quiet_NaN(), 0.2));
	driverRos.cmdVelCallback(twist(std::numeric_limits<double>::infinity(), 0.0, 0.0));

	auto& targets = driverRos.recorder()->targets;
	ASSERT_EQ(targets.size(), 2u);
	for (const Velocity& v : targets) {
		EXPECT_EQ(v.vx, 0.0);
		EXPECT_EQ(v.vy, 0.0);
		EXPECT_EQ(v.va, 0.0);
	}
}

TEST_F(PlatformDriverROSWatchdog, noZeroBeforeFirstCommand) {
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	driverRos.step();

	EXPECT_TRUE(driverRos.recorder()->targets.empty());
}

// A driver that moves a robot starts on a valid configuration or not at all.
class PlatformDriverROSStartup : public ::testing::TestWithParam<rclcpp::Parameter> {
protected:
	static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
	static void TearDownTestSuite() { rclcpp::shutdown(); }

	bool initWith(const std::vector<rclcpp::Parameter>& params) {
		rclcpp::NodeOptions options;
		options.parameter_overrides(params);
		auto node = std::make_shared<rclcpp::Node>("platform_driver_startup_test", options);
		TestablePlatformDriverROS driverRos;
		return driverRos.init(node, "");
	}
};

TEST_F(PlatformDriverROSStartup, aValidConfigurationStarts) {
	EXPECT_TRUE(initWith(oneWheel()));
}

TEST_P(PlatformDriverROSStartup, refusesToStart) {
	EXPECT_FALSE(initWith(with(oneWheel(), GetParam()))) << GetParam().get_name() << " = "
		<< GetParam().value_to_string();
}

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

INSTANTIATE_TEST_SUITE_P(InvalidValues, PlatformDriverROSStartup, ::testing::Values(
	// A mistyped timeout (e.g. milliseconds) must not become minutes of stale travel.
	rclcpp::Parameter("cmd_vel_timeout", 200.0),
	rclcpp::Parameter("cmd_vel_timeout", 0.0),
	rclcpp::Parameter("cmd_vel_timeout", -0.1),
	rclcpp::Parameter("cmd_vel_timeout", kInf),
	rclcpp::Parameter("cmd_vel_timeout", kNaN),
	// A negative limit drives with no command; a zero deceleration never stops.
	rclcpp::Parameter("vlin_max", -1.0),
	rclcpp::Parameter("vlin_max", kNaN),
	rclcpp::Parameter("va_max", 0.0),
	rclcpp::Parameter("vlin_acc_max", kInf),
	rclcpp::Parameter("vlin_dec_max", 0.0),
	rclcpp::Parameter("va_dec_max", -2.0),
	rclcpp::Parameter("stop_vlin_dec", 0.0),
	rclcpp::Parameter("stop_timeout", 0.1),
	rclcpp::Parameter("num_wheels", 0),
	rclcpp::Parameter("wheel0.model", "KD999"),
	rclcpp::Parameter("wheel0.ethercat_number", 0),
	rclcpp::Parameter("wheel_models.KD165.diameter", 0.0),
	rclcpp::Parameter("wheel_models.KD165.currentlimit", kNaN),
	rclcpp::Parameter("wheel_slew_rate", -1.0),
	rclcpp::Parameter("odom_stale_ticks", 0),
	rclcpp::Parameter("stale_twist_covariance", kInf)));

} // namespace
