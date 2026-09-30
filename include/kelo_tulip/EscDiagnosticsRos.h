/******************************************************************************
 * This software is published under the same dual-license as the rest of
 * kelo_tulip: GNU Lesser General Public License LGPL 2.1 and BSD license.
 ******************************************************************************/

#ifndef KELOTULIP_ESCDIAGNOSTICSROS_H
#define KELOTULIP_ESCDIAGNOSTICSROS_H

#include <functional>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "kelo_tulip/EscDiagnostics.h"
#include "rclcpp/rclcpp.hpp"

namespace kelo {

// One DiagnosticStatus per slave: OK while its counters are still, WARN when
// any grew since the previous read, ERROR when the slave could not be read.
// `values` carry the absolute counters (per port where there is one) and what
// grew, so a rising counter can be tied to a port and thus a cable segment.
diagnostic_msgs::msg::DiagnosticArray buildEscDiagnosticArray(const EscSnapshot& snapshot,
	const rclcpp::Time& stamp);

// Publishes each new snapshot once, from the ROS thread.
class EscDiagnosticsPublisher {
public:
	using SnapshotSource = std::function<EscSnapshot()>;

	EscDiagnosticsPublisher(rclcpp::Node::SharedPtr node, SnapshotSource source);
	void publishIfNew();

private:
	rclcpp::Node::SharedPtr node_;
	SnapshotSource source_;
	rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
	std::uint64_t lastSequence_ = 0;
};

}  // namespace kelo

#endif  // KELOTULIP_ESCDIAGNOSTICSROS_H
