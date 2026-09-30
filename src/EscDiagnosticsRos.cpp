#include "kelo_tulip/EscDiagnosticsRos.h"

#include <string>

namespace kelo {

namespace {

using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;

void addValue(DiagnosticStatus& status, const std::string& key, const std::string& value) {
	KeyValue kv;
	kv.key = key;
	kv.value = value;
	status.values.push_back(kv);
}

template <typename Counters, typename Deltas>
void addPortValues(DiagnosticStatus& status, const char* name, const Counters& counters, const Deltas& deltas) {
	for (int port = 0; port < kEscPorts; port++) {
		const std::string suffix = std::string(name) + "_p" + std::to_string(port);
		addValue(status, suffix, std::to_string(counters[port]));
		addValue(status, "delta_" + suffix, std::to_string(deltas[port]));
	}
}

DiagnosticStatus buildSlaveStatus(const EscSlaveReading& reading, bool afterEpisode) {
	DiagnosticStatus status;
	status.name = "ethercat/slave_" + std::to_string(reading.slave);
	status.hardware_id = "ethercat_slave_" + std::to_string(reading.slave);
	if (!reading.ok) {
		status.level = DiagnosticStatus::ERROR;
		status.message = "ESC error counters could not be read";
	} else if (reading.delta.any()) {
		status.level = DiagnosticStatus::WARN;
		status.message = describeEscDelta(reading.delta);
	} else {
		status.level = DiagnosticStatus::OK;
		status.message = "no new errors";
	}
	addValue(status, "after_episode", afterEpisode ? "true" : "false");
	if (!reading.ok)
		return status;

	addPortValues(status, "invalid_frame", reading.counters.invalidFrame, reading.delta.invalidFrame);
	addPortValues(status, "rx_error", reading.counters.rxError, reading.delta.rxError);
	addPortValues(status, "forwarded_rx_error", reading.counters.forwardedRxError, reading.delta.forwardedRxError);
	addPortValues(status, "lost_link", reading.counters.lostLink, reading.delta.lostLink);
	addValue(status, "processing_unit_error", std::to_string(reading.counters.processingUnitError));
	addValue(status, "delta_processing_unit_error", std::to_string(reading.delta.processingUnitError));
	addValue(status, "pdi_error", std::to_string(reading.counters.pdiError));
	addValue(status, "delta_pdi_error", std::to_string(reading.delta.pdiError));
	addValue(status, "first_read", reading.delta.firstRead ? "true" : "false");
	addValue(status, "restarted", reading.delta.restarted ? "true" : "false");
	addValue(status, "saturated", reading.delta.saturated ? "true" : "false");
	return status;
}

}  // namespace

diagnostic_msgs::msg::DiagnosticArray buildEscDiagnosticArray(const EscSnapshot& snapshot,
	const rclcpp::Time& stamp) {
	diagnostic_msgs::msg::DiagnosticArray array;
	array.header.stamp = stamp;
	for (const EscSlaveReading& reading : snapshot.slaves)
		array.status.push_back(buildSlaveStatus(reading, snapshot.afterEpisode));
	return array;
}

EscDiagnosticsPublisher::EscDiagnosticsPublisher(rclcpp::Node::SharedPtr node, SnapshotSource source)
	: node_(node)
	, source_(source)
	, publisher_(node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/ethercat_diagnostics", 10))
{
}

void EscDiagnosticsPublisher::publishIfNew() {
	const EscSnapshot snapshot = source_();
	if (snapshot.sequence == 0 || snapshot.sequence == lastSequence_)
		return;
	lastSequence_ = snapshot.sequence;
	publisher_->publish(buildEscDiagnosticArray(snapshot, node_->now()));
}

}  // namespace kelo
