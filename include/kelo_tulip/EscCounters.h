/******************************************************************************
 * This software is published under the same dual-license as the rest of
 * kelo_tulip: GNU Lesser General Public License LGPL 2.1 and BSD license.
 ******************************************************************************/

#ifndef KELOTULIP_ESCCOUNTERS_H
#define KELOTULIP_ESCCOUNTERS_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace kelo {

constexpr int kEscPorts = 4;
// One read covers every error counter: 0x0300..0x0313.
constexpr std::uint16_t kEscErrorRegBase = 0x0300;
constexpr std::uint16_t kEscErrorRegLength = 0x14;

// The ESC's 8-bit error counters (ETG.1000 register map). They saturate at 255
// and are cleared only by a write, which this driver never does.
struct EscCounters {
	std::array<std::uint8_t, kEscPorts> invalidFrame{};
	std::array<std::uint8_t, kEscPorts> rxError{};
	std::array<std::uint8_t, kEscPorts> forwardedRxError{};
	std::uint8_t processingUnitError = 0;
	std::uint8_t pdiError = 0;
	std::array<std::uint8_t, kEscPorts> lostLink{};
};

EscCounters decodeEscCounters(const std::uint8_t* raw);

// What changed since the previous read of the same slave.
struct EscCounterDelta {
	std::array<unsigned, kEscPorts> invalidFrame{};
	std::array<unsigned, kEscPorts> rxError{};
	std::array<unsigned, kEscPorts> forwardedRxError{};
	unsigned processingUnitError = 0;
	unsigned pdiError = 0;
	std::array<unsigned, kEscPorts> lostLink{};
	bool firstRead = false;
	// A counter went backwards: the slave was power-cycled, its counters restarted.
	bool restarted = false;
	// A counter sits at 255, so further errors on it are no longer visible.
	bool saturated = false;

	bool any() const;
};

class EscCounterTracker {
public:
	EscCounterDelta update(std::size_t slave, const EscCounters& current);

private:
	std::vector<EscCounters> previous_;
	std::vector<bool> seen_;
};

std::string describeEscDelta(const EscCounterDelta& delta);

}  // namespace kelo

#endif  // KELOTULIP_ESCCOUNTERS_H
