#include <gtest/gtest.h>

#include <cstdint>
#include <memory>

#include "fake_kelo_bus.h"
#include "kelo_tulip/PlatformDriver.h"

namespace {

using kelo::SampleInstant;
using kelo_test::FakeBus;
using kelo_test::kWheels;

const int64_t MS = 1000000;
const int64_t T0 = 1760000000LL * 1000000000LL;
const uint64_t S0 = 7000000000ULL;  // the drives' own counters

// The host clock stands still during a cycle, so a cycle's arrival and its
// send instant are the same reading.
class HostClockedDriver : public kelo_test::ClockedDriver {
public:
	using kelo_test::ClockedDriver::ClockedDriver;
	int64_t hostNowNs() const override { return hostNs; }
	int64_t hostNs = T0;
};

class PlatformDriverSampleInstant : public ::testing::Test {
protected:
	void SetUp() override {
		driver = std::make_unique<HostClockedDriver>(kelo_test::fakeWheelConfigs(), kelo_test::fakeWheelData());
		ASSERT_TRUE(driver->initEtherCAT(bus.slaves(), kWheels));
		bus.update();
	}

	// One EtherCAT cycle at host time t, reading the frame sent at the end of
	// the previous one.
	void cycle(int64_t t) {
		driver->hostNs = t;
		driver->clockMs += 1.0;
		driver->step();
		bus.update();
	}

	void report(int wheel, uint64_t sensorTs) { bus.report(wheel).sensor_ts = sensorTs; }

	void reportAll(uint64_t sensorTs) {
		for (int w = 0; w < kWheels; w++)
			report(w, sensorTs);
	}

	SampleInstant instant(int wheel) {
		SampleInstant out;
		EXPECT_TRUE(driver->getSampleInstant(wheel, out)) << "wheel " << wheel;
		return out;
	}

	FakeBus bus;
	std::unique_ptr<HostClockedDriver> driver;
};

TEST_F(PlatformDriverSampleInstant, nothingIsCapturedBeforeTheFirstSample) {
	cycle(T0);
	SampleInstant out;
	EXPECT_FALSE(driver->getSampleInstant(0, out));
}

TEST_F(PlatformDriverSampleInstant, theFirstFrameHasOnlyItsArrival) {
	reportAll(S0);
	cycle(T0);
	EXPECT_EQ(instant(0).sensorTs, S0);
	EXPECT_EQ(instant(0).hostNs, T0);
}

TEST_F(PlatformDriverSampleInstant, aNewSampleCarriesTheInstantItsFrameWasSent) {
	reportAll(S0);
	cycle(T0);
	reportAll(S0 + MS);
	cycle(T0 + MS);
	EXPECT_EQ(instant(0).sensorTs, S0 + MS);
	EXPECT_EQ(instant(0).hostNs, T0);
}

TEST_F(PlatformDriverSampleInstant, aRepeatedSampleKeepsItsInstant) {
	reportAll(S0);
	cycle(T0);
	reportAll(S0 + MS);
	cycle(T0 + MS);
	cycle(T0 + 2 * MS);
	cycle(T0 + 3 * MS);
	EXPECT_EQ(instant(1).sensorTs, S0 + MS);
	EXPECT_EQ(instant(1).hostNs, T0);
}

TEST_F(PlatformDriverSampleInstant, wheelsInOneFrameShareItsInstant) {
	for (int w = 0; w < kWheels; w++)
		report(w, S0 + 1000 * w);
	cycle(T0);
	for (int w = 0; w < kWheels; w++)
		report(w, S0 + 1000 * w + MS);
	cycle(T0 + MS);
	for (int w = 0; w < kWheels; w++) {
		EXPECT_EQ(instant(w).sensorTs, S0 + 1000 * w + MS) << "wheel " << w;
		EXPECT_EQ(instant(w).hostNs, T0) << "wheel " << w;
	}
}

TEST_F(PlatformDriverSampleInstant, aWheelThatMissedAFrameKeepsItsOlderSample) {
	reportAll(S0);
	cycle(T0);
	reportAll(S0 + MS);
	cycle(T0 + MS);
	reportAll(S0 + 2 * MS);
	report(2, S0 + MS);
	cycle(T0 + 2 * MS);
	EXPECT_EQ(instant(0).hostNs, T0 + MS);
	EXPECT_EQ(instant(2).sensorTs, S0 + MS);
	EXPECT_EQ(instant(2).hostNs, T0);
}

TEST_F(PlatformDriverSampleInstant, afterAReinitTheFirstFrameHasOnlyItsArrival) {
	reportAll(S0);
	cycle(T0);
	// The re-init sends its own frames; the last cycle's send instant is not theirs.
	ASSERT_TRUE(driver->initEtherCAT(bus.slaves(), kWheels));
	reportAll(S0 + 3000 * MS);
	cycle(T0 + 3000 * MS);
	EXPECT_EQ(instant(0).hostNs, T0 + 3000 * MS);
}

TEST_F(PlatformDriverSampleInstant, aWheelOutOfRangeHasNoInstant) {
	reportAll(S0);
	cycle(T0);
	SampleInstant out;
	EXPECT_FALSE(driver->getSampleInstant(kWheels, out));
}

}  // namespace
