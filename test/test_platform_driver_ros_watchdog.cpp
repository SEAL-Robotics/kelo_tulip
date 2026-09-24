#include <gtest/gtest.h>

#include <chrono>
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

protected:
	kelo::PlatformDriver* createDriver() override {
		return new RecordingPlatformDriver(wheelConfigs, wheelData);
	}
};

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
		options.parameter_overrides({
			rclcpp::Parameter("num_wheels", 0),
			rclcpp::Parameter("cmd_vel_timeout", 0.05),
		});
		node = std::make_shared<rclcpp::Node>("platform_driver_watchdog_test", options);
		ASSERT_TRUE(driverRos.init(node, ""));
	}

	rclcpp::Node::SharedPtr node;
	TestablePlatformDriverROS driverRos;
};

TEST_F(PlatformDriverROSWatchdog, readsTimeoutParameter) {
	EXPECT_DOUBLE_EQ(node->get_parameter("cmd_vel_timeout").as_double(), 0.05);
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

TEST_F(PlatformDriverROSWatchdog, noZeroBeforeFirstCommand) {
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	driverRos.step();

	EXPECT_TRUE(driverRos.recorder()->targets.empty());
}

} // namespace
