#include "kelo_tulip/EscCounters.h"

#include <sstream>

namespace kelo {

namespace {

constexpr std::size_t kInvalidFrameOffset = 0x00;  // 0x0300, then RX error at +1 per port
constexpr std::size_t kForwardedRxOffset = 0x08;
constexpr std::size_t kProcessingUnitOffset = 0x0C;
constexpr std::size_t kPdiOffset = 0x0D;
constexpr std::size_t kLostLinkOffset = 0x10;
constexpr unsigned kSaturated = 255;

// Growth of one counter; a smaller value than before can only mean a restart.
unsigned grow(std::uint8_t previous, std::uint8_t current, bool& restarted) {
	if (current < previous) {
		restarted = true;
		return current;
	}
	return static_cast<unsigned>(current - previous);
}

template <typename Array>
void appendPorts(std::ostringstream& text, const char* name, const Array& deltas) {
	for (int port = 0; port < kEscPorts; port++) {
		if (deltas[port] == 0)
			continue;
		if (text.tellp() > 0)
			text << ", ";
		text << name << "[p" << port << "] +" << deltas[port];
	}
}

}  // namespace

EscCounters decodeEscCounters(const std::uint8_t* raw) {
	EscCounters c;
	for (int port = 0; port < kEscPorts; port++) {
		c.invalidFrame[port] = raw[kInvalidFrameOffset + 2 * port];
		c.rxError[port] = raw[kInvalidFrameOffset + 2 * port + 1];
		c.forwardedRxError[port] = raw[kForwardedRxOffset + port];
		c.lostLink[port] = raw[kLostLinkOffset + port];
	}
	c.processingUnitError = raw[kProcessingUnitOffset];
	c.pdiError = raw[kPdiOffset];
	return c;
}

bool EscCounterDelta::any() const {
	if (processingUnitError || pdiError)
		return true;
	for (int port = 0; port < kEscPorts; port++)
		if (invalidFrame[port] || rxError[port] || forwardedRxError[port] || lostLink[port])
			return true;
	return false;
}

EscCounterDelta EscCounterTracker::update(std::size_t slave, const EscCounters& current) {
	if (slave >= previous_.size()) {
		previous_.resize(slave + 1);
		seen_.resize(slave + 1, false);
	}
	EscCounterDelta delta;
	for (int port = 0; port < kEscPorts; port++) {
		if (current.invalidFrame[port] == kSaturated || current.rxError[port] == kSaturated ||
			current.forwardedRxError[port] == kSaturated || current.lostLink[port] == kSaturated)
			delta.saturated = true;
	}
	if (current.processingUnitError == kSaturated || current.pdiError == kSaturated)
		delta.saturated = true;

	if (!seen_[slave]) {
		seen_[slave] = true;
		delta.firstRead = true;
		previous_[slave] = current;
		return delta;
	}

	const EscCounters& before = previous_[slave];
	for (int port = 0; port < kEscPorts; port++) {
		delta.invalidFrame[port] = grow(before.invalidFrame[port], current.invalidFrame[port], delta.restarted);
		delta.rxError[port] = grow(before.rxError[port], current.rxError[port], delta.restarted);
		delta.forwardedRxError[port] =
			grow(before.forwardedRxError[port], current.forwardedRxError[port], delta.restarted);
		delta.lostLink[port] = grow(before.lostLink[port], current.lostLink[port], delta.restarted);
	}
	delta.processingUnitError = grow(before.processingUnitError, current.processingUnitError, delta.restarted);
	delta.pdiError = grow(before.pdiError, current.pdiError, delta.restarted);
	previous_[slave] = current;
	return delta;
}

std::string describeEscDelta(const EscCounterDelta& delta) {
	std::ostringstream text;
	appendPorts(text, "invalid_frame", delta.invalidFrame);
	appendPorts(text, "rx_error", delta.rxError);
	appendPorts(text, "forwarded_rx_error", delta.forwardedRxError);
	appendPorts(text, "lost_link", delta.lostLink);
	if (delta.processingUnitError) {
		if (text.tellp() > 0)
			text << ", ";
		text << "processing_unit_error +" << delta.processingUnitError;
	}
	if (delta.pdiError) {
		if (text.tellp() > 0)
			text << ", ";
		text << "pdi_error +" << delta.pdiError;
	}
	return text.str();
}

}  // namespace kelo
