#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "kelo_tulip/PlatformDriverROS.h"
#include "rclcpp/rclcpp.hpp"

namespace {

const int64_t MS = 1000000;
// Far from the test's own clock: a stamp taken from the ROS loop would show.
const int64_t T0 = 1760000000LL * 1000000000LL;
const uint64_t S0 = 7000000000ULL;
const uint64_t WINDOW_NS = 50 * MS;

// The EtherCAT side on a host clock the test sets.
class HostClockedPlatformDriver : public kelo::PlatformDriver {
public:
	using kelo::PlatformDriver::PlatformDriver;
	int64_t hostNowNs() const override { return hostNs; }
	void setTargetVelocity(double, double, double) override {}
	int64_t hostNs = T0;
};

class OdometryDriverROS : public kelo::PlatformDriverROS {
public:
	HostClockedPlatformDriver* bus() { return static_cast<HostClockedPlatformDriver*>(driver); }

	void attachFakeBus() {
		std::memset(slaves, 0, sizeof(slaves));
		std::memset(&inputs, 0, sizeof(inputs));
		std::memset(&outputs, 0, sizeof(outputs));
		slaves[1].inputs = reinterpret_cast<uint8*>(&inputs);
		slaves[1].outputs = reinterpret_cast<uint8*>(&outputs);
		ASSERT_TRUE(bus()->initEtherCAT(slaves, 1));
	}

	// The frame sent at sentNs carries this sample; the next cycle reads it.
	void frame(int64_t sentNs, uint64_t sensorTs, float encoder1, float encoder2) {
		bus()->hostNs = sentNs;
		bus()->step();
		inputs.sensor_ts = sensorTs;
		inputs.encoder_1 = encoder1;
		inputs.encoder_2 = encoder2;
		bus()->hostNs = sentNs + MS;
		bus()->step();
	}

	txpdo1_t inputs;

protected:
	kelo::PlatformDriver* createDriver() override {
		return new HostClockedPlatformDriver(wheelConfigs, wheelData);
	}

	ec_slavet slaves[2];
	rxpdo1_t outputs;
};

std::vector<rclcpp::Parameter> oneWheel() {
	return {
		rclcpp::Parameter("num_wheels", 1),
		rclcpp::Parameter("wheel_models.list", std::vector<std::string>{"KD165"}),
		rclcpp::Parameter("wheel0.model", "KD165"),
		rclcpp::Parameter("wheel0.ethercat_number", 1),
		rclcpp::Parameter("wheel0.x", 0.5),
	};
}

int64_t ns(const builtin_interfaces::msg::Time& t) {
	return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec;
}

class PlatformDriverROSOdometry : public ::testing::Test {
protected:
	static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
	static void TearDownTestSuite() { rclcpp::shutdown(); }

	void SetUp() override {
		rclcpp::NodeOptions options;
		options.parameter_overrides(oneWheel());
		// Delivered in-process, so the test waits on no discovery.
		options.use_intra_process_comms(true);
		node = std::make_shared<rclcpp::Node>("platform_driver_odometry_test", options);
		odomSub = node->create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
			[this](nav_msgs::msg::Odometry::SharedPtr msg) { received.push_back(*msg); });
		ASSERT_TRUE(driverRos.init(node, ""));
		driverRos.attachFakeBus();
	}

	nav_msgs::msg::Odometry rosStep() {
		const size_t before = received.size();
		driverRos.step();
		rclcpp::spin_some(node);
		EXPECT_EQ(received.size(), before + 1);
		return received.empty() ? nav_msgs::msg::Odometry() : received.back();
	}

	rclcpp::Node::SharedPtr node;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
	std::vector<nav_msgs::msg::Odometry> received;
	OdometryDriverROS driverRos;
};

TEST_F(PlatformDriverROSOdometry, withoutAnySampleTheRosClockStamps) {
	const int64_t now = node->get_clock()->now().nanoseconds();
	const nav_msgs::msg::Odometry odom = rosStep();
	EXPECT_NEAR(static_cast<double>(ns(odom.header.stamp) - now), 0.0, 5e9);
	EXPECT_GE(odom.twist.covariance[0], 1e5);
}

TEST_F(PlatformDriverROSOdometry, eachMessageIsStampedAtTheInstantItsTwistAndPoseDescribe) {
	const int64_t sentA = T0;
	const int64_t sentB = T0 + 50 * MS + 300000;

	// First data: a resync, measured with the drives' own (zero) velocities.
	driverRos.frame(sentA, S0, 0.0f, 0.0f);
	const nav_msgs::msg::Odometry first = rosStep();
	EXPECT_EQ(ns(first.header.stamp), sentA);

	// 50 ms of encoder travel: the mean velocity over the window, so the
	// window's midpoint, and the pose half way through it.
	driverRos.frame(sentB, S0 + WINDOW_NS, 0.5f, -0.5f);
	const nav_msgs::msg::Odometry window = rosStep();
	EXPECT_EQ(ns(window.header.stamp), sentB - static_cast<int64_t>(WINDOW_NS / 2));
	EXPECT_GT(std::fabs(window.twist.twist.linear.x), 0.1);
	EXPECT_LT(window.twist.covariance[0], 1.0);

	// No new sample: held at the window's end, where the integrated pose is.
	const nav_msgs::msg::Odometry held = rosStep();
	EXPECT_EQ(ns(held.header.stamp), sentB);
	EXPECT_DOUBLE_EQ(held.twist.twist.linear.x, 0.0);
	EXPECT_GE(held.twist.covariance[0], 1e5);

	const double travelled = held.pose.pose.position.x - first.pose.pose.position.x;
	EXPECT_GT(std::fabs(travelled), 1e-3);
	EXPECT_NEAR(window.pose.pose.position.x, first.pose.pose.position.x + 0.5 * travelled, 1e-9);
	EXPECT_NEAR(travelled, window.twist.twist.linear.x * 1e-9 * WINDOW_NS, 1e-9);
}

TEST_F(PlatformDriverROSOdometry, aSampleWithoutACapturedInstantIsStampedByTheRosClock) {
	// Data in the process image that no EtherCAT cycle has seen yet.
	driverRos.inputs.sensor_ts = S0;
	const int64_t before = node->get_clock()->now().nanoseconds();
	const nav_msgs::msg::Odometry odom = rosStep();
	const int64_t after = node->get_clock()->now().nanoseconds();
	EXPECT_GE(ns(odom.header.stamp), before);
	EXPECT_LE(ns(odom.header.stamp), after);
}

TEST_F(PlatformDriverROSOdometry, theStampDoesNotDependOnWhenTheRosLoopRuns) {
	driverRos.frame(T0, S0, 0.0f, 0.0f);
	rosStep();
	driverRos.frame(T0 + 50 * MS, S0 + WINDOW_NS, 0.2f, -0.2f);
	// However late the ROS loop reads the sample, it describes the same instant.
	rclcpp::sleep_for(std::chrono::milliseconds(20));
	EXPECT_EQ(ns(rosStep().header.stamp), T0 + 25 * MS);
}

}  // namespace
