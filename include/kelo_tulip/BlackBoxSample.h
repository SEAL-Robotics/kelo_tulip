/******************************************************************************
 * This software is published under the same dual-license as the rest of
 * kelo_tulip: GNU Lesser General Public License LGPL 2.1 and BSD license.
 ******************************************************************************/

#ifndef KELOTULIP_BLACKBOXSAMPLE_H
#define KELOTULIP_BLACKBOXSAMPLE_H

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <vector>

namespace kelo {

constexpr std::size_t kMaxBlackBoxWheels = 8;

constexpr std::uint8_t kWheelFlagLinkUp = 1u << 0;
constexpr std::uint8_t kWheelFlagEnabled = 1u << 1;

// One wheel's slice of one EtherCAT cycle: what the drive reported and what
// was commanded back. Plain data, sized in multiples of 8 bytes, so the
// realtime thread copies it into a preallocated ring without allocating.
struct WheelSample {
	std::uint64_t sensorTs;
	float voltageBus;
	float currentIn;
	float current1q;
	float current2q;
	float setpoint1;
	float setpoint2;
	float limit1p;
	float limit2p;
	std::uint16_t status1;
	std::uint16_t status2;
	std::uint16_t command1;
	std::uint8_t wheelState;
	std::uint8_t flags;
};

struct CycleSample {
	std::uint64_t cycle;
	std::int64_t monotonicNs;
	std::int32_t wkc;
	std::int32_t expectedWkc;
	std::uint8_t driverState;
	std::uint8_t wheelCount;
	std::uint8_t reserved[6];
	WheelSample wheels[kMaxBlackBoxWheels];
};

// One header line, then one row per sample. `t_ms` is relative to the
// sample whose cycle equals triggerCycle, which is also the only row with
// trigger=1.
void writeBlackBoxCsv(std::ostream& out, const std::vector<CycleSample>& samples, std::size_t wheelCount,
	std::uint64_t triggerCycle);

}  // namespace kelo

#endif  // KELOTULIP_BLACKBOXSAMPLE_H
