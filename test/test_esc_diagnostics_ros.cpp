#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "kelo_tulip/EscDiagnosticsRos.h"

using kelo::buildEscDiagnosticArray;
using kelo::EscSlaveReading;
using kelo::EscSnapshot;

namespace {

std::string valueOf(const diagnostic_msgs::msg::DiagnosticStatus& status, const std::string& key) {
	for (const auto& kv : status.values)
		if (kv.key == key)
			return kv.value;
	return "<missing>";
}

EscSlaveReading reading(int slave, bool ok) {
	EscSlaveReading r;
	r.slave = slave;
	r.ok = ok;
	return r;
}

TEST(BuildEscDiagnosticArray, hasOneStatusPerSlaveNamedAfterIt) {
	EscSnapshot snap;
	snap.slaves = {reading(1, true), reading(4, true)};

	const auto array = buildEscDiagnosticArray(snap, rclcpp::Time(5, 0));

	ASSERT_EQ(array.status.size(), 2u);
	EXPECT_EQ(array.status[0].name, "ethercat/slave_1");
	EXPECT_EQ(array.status[1].name, "ethercat/slave_4");
	EXPECT_EQ(array.header.stamp.sec, 5);
}

TEST(BuildEscDiagnosticArray, aQuietSlaveIsOk) {
	EscSnapshot snap;
	snap.slaves = {reading(1, true)};

	const auto array = buildEscDiagnosticArray(snap, rclcpp::Time(0, 0));

	ASSERT_EQ(array.status.size(), 1u);
	EXPECT_EQ(array.status[0].level, diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST(BuildEscDiagnosticArray, aGrowingCounterIsAWarningNamingThePort) {
	EscSnapshot snap;
	EscSlaveReading r = reading(2, true);
	r.counters.rxError[1] = 17;
	r.counters.lostLink[0] = 1;
	r.delta.rxError[1] = 4;
	snap.slaves = {r};

	const auto array = buildEscDiagnosticArray(snap, rclcpp::Time(0, 0));
	ASSERT_EQ(array.status.size(), 1u);
	const auto& status = array.status[0];

	EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
	EXPECT_EQ(valueOf(status, "rx_error_p1"), "17");
	EXPECT_EQ(valueOf(status, "delta_rx_error_p1"), "4");
	EXPECT_EQ(valueOf(status, "lost_link_p0"), "1");
	EXPECT_EQ(valueOf(status, "delta_rx_error_p0"), "0");
	EXPECT_NE(status.message.find("rx_error[p1] +4"), std::string::npos) << status.message;
}

TEST(BuildEscDiagnosticArray, aSlaveThatCouldNotBeReadIsAnError) {
	EscSnapshot snap;
	snap.slaves = {reading(3, false)};

	const auto array = buildEscDiagnosticArray(snap, rclcpp::Time(0, 0));

	ASSERT_EQ(array.status.size(), 1u);
	EXPECT_EQ(array.status[0].level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

TEST(BuildEscDiagnosticArray, markAReadTakenAfterACommunicationError) {
	EscSnapshot snap;
	snap.afterEpisode = true;
	snap.slaves = {reading(1, true)};

	const auto array = buildEscDiagnosticArray(snap, rclcpp::Time(0, 0));

	ASSERT_EQ(array.status.size(), 1u);
	EXPECT_EQ(valueOf(array.status[0], "after_episode"), "true");
}

TEST(EscDiagnosticsPublisher, publishesEachSnapshotOnceAndNothingBeforeThe) {
	rclcpp::init(0, nullptr);
	{
		auto node = std::make_shared<rclcpp::Node>("esc_diag_test");
		auto listener = std::make_shared<rclcpp::Node>("esc_diag_listener");
		int received = 0;
		auto sub = listener->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
			"/esc_diag_test/ethercat_diagnostics", 10,
			[&received](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr) { received++; });
		EscSnapshot snap;
		snap.sequence = 0;
		kelo::EscDiagnosticsPublisher publisher(node, [&snap] { return snap; });

		publisher.publishIfNew();  // sequence 0: nothing read yet
		snap.sequence = 1;
		snap.slaves = {reading(1, true)};
		publisher.publishIfNew();
		publisher.publishIfNew();  // same snapshot again

		for (int i = 0; i < 50 && received < 1; i++) {
			rclcpp::spin_some(listener);
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
		for (int i = 0; i < 5; i++) {
			rclcpp::spin_some(listener);
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
		EXPECT_EQ(received, 1);
	}
	rclcpp::shutdown();
}

}  // namespace
