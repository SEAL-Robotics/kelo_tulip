#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "kelo_tulip/WheelRecovery.h"

using kelo::WheelRecoveryConfig;
using kelo::WheelRecoveryMachine;
using kelo::WheelState;

namespace {

// status1 63 with status2 2051 is healthy; everything else needs recovery.
struct Scenario {
	WheelRecoveryMachine machine{WheelRecoveryConfig(), 0.0};
	double nowMs = 0.0;
	unsigned int events = 0;
	bool everAllowedEnableWhileFaulty = false;

	// Advance to `untilMs` in 1 ms cycles with a fixed input.
	WheelRecoveryMachine::Output run(double untilMs, bool linkUp, bool needsRecovery, bool sane = true,
		bool operatorEnable = true, bool safeTorqueOff = false, bool frameValid = true) {
		WheelRecoveryMachine::Output out{true, false, 0};
		while (nowMs < untilMs) {
			nowMs += 1.0;
			out = machine.step({nowMs, operatorEnable, needsRecovery, linkUp, sane, safeTorqueOff, frameValid});
			events |= out.events;
		}
		return out;
	}
	bool saw(kelo::RecoveryEvent e) const { return (events & e) != 0; }
};

// A drive on an emergency pair. Open, it is in safe torque off. Closed, it
// keeps its errors latched until it has been disabled and then enabled again;
// with ossdOkLatched it keeps OSSD_OK clear until then too.
struct PairDrive {
	bool open = false;
	bool ossdOkLatched = false;
	bool latched = false;
	bool disabledSinceClose = false;

	bool safeTorqueOff() const { return open || (latched && ossdOkLatched); }
	bool needsRecovery() const { return open || latched; }
	void command(bool enabled) {
		if (open) {
			latched = true;
			disabledSinceClose = false;
		} else if (!enabled) {
			disabledSinceClose = true;
		} else if (disabledSinceClose) {
			latched = false;
		}
	}
};

struct PairScenario {
	WheelRecoveryMachine machine{WheelRecoveryConfig(), 0.0};
	PairDrive drive;
	double nowMs = 0.0;
	unsigned int events = 0;

	WheelRecoveryMachine::Output run(double untilMs, bool operatorEnable = true) {
		WheelRecoveryMachine::Output out{true, false, 0};
		while (nowMs < untilMs) {
			nowMs += 1.0;
			const bool faulty = drive.needsRecovery();
			out = machine.step({nowMs, operatorEnable, faulty, true, !faulty, drive.safeTorqueOff(), true});
			events |= out.events;
			drive.command(operatorEnable && out.allowsEnable);
		}
		return out;
	}
	bool saw(kelo::RecoveryEvent e) const { return (events & e) != 0; }
};

TEST(WheelInSafeTorqueOff, readsAnOpenEmergencyPairFromAClearedOssdOk) {
	// 5177: MOTOR_STOP, OSSD_OK clear, EXT_DISABLE_ERR, ECAT_WDOG_ERR, as all
	// four drives reported with the E-stop pressed.
	EXPECT_TRUE(kelo::wheelInSafeTorqueOff(5177));
	EXPECT_TRUE(kelo::wheelInSafeTorqueOff(63 & ~0x0004));
}

TEST(WheelInSafeTorqueOff, aClosedPairWithItsErrorsStillLatchedIsNotOpen) {
	// 1085: OSSD_OK set, EXT_DISABLE_ERR latched until a disable-enable.
	EXPECT_FALSE(kelo::wheelInSafeTorqueOff(1085));
	EXPECT_FALSE(kelo::wheelInSafeTorqueOff(63));
	EXPECT_FALSE(kelo::wheelInSafeTorqueOff(61));
	EXPECT_FALSE(kelo::wheelInSafeTorqueOff(125));
}

TEST(WheelInSafeTorqueOff, aZeroedProcessImageSaysNothing) {
	// A lost slave's inputs are zeroed: B4 and B5, set in every real frame, are clear.
	EXPECT_FALSE(kelo::wheelInSafeTorqueOff(0));
}

TEST(WheelRecoveryMachine, anOpenEmergencyPairHoldsTheWheelWithoutSpendingAttempts) {
	PairScenario s;
	s.run(100);
	s.drive.open = true;
	auto out = s.run(150);
	EXPECT_EQ(s.machine.state(), kelo::WHEEL_SAFE_TORQUE_OFF);
	EXPECT_FALSE(out.allowsEnable);
	EXPECT_TRUE(out.holdAtZero);
	EXPECT_TRUE(s.saw(kelo::RECOVERY_EVENT_SAFE_TORQUE_OFF));

	out = s.run(60000);
	EXPECT_FALSE(s.machine.failed());
	EXPECT_EQ(s.machine.attempts(), 0u);
	EXPECT_TRUE(out.holdAtZero);
	EXPECT_FALSE(s.saw(kelo::RECOVERY_EVENT_GAVE_UP));
}

TEST(WheelRecoveryMachine, aPairThatClosesDuringARetestIsRetestedOnceMoreWithoutAnAttempt) {
	PairScenario s;
	s.run(100);
	s.drive.open = true;
	// The first retest starts a second after the pair opened; close inside it.
	while (s.machine.state() != kelo::WHEEL_STATUS_RECOVERY_WAITIING_FOR_NORMAL_OPERATION)
		s.run(s.nowMs + 1);
	s.drive.open = false;
	s.run(s.nowMs + 500);
	EXPECT_FALSE(s.drive.latched);
	EXPECT_EQ(s.machine.state(), kelo::WHEEL_NORMAL_OPERATION);
	EXPECT_EQ(s.machine.attempts(), 0u);
}

TEST(WheelRecoveryMachine, aClosedPairThatLeavesAnotherFaultFallsBackToCountedRecovery) {
	Scenario s;
	s.run(100, true, false);
	s.run(2000, true, true, false, true, true);
	// Pair closed, but the drive keeps an error a disable-enable does not clear.
	s.run(6000, true, true, false);
	EXPECT_TRUE(s.machine.failed());
	EXPECT_TRUE(s.saw(kelo::RECOVERY_EVENT_GAVE_UP));
}

TEST(WheelRecoveryMachine, aClosedPairIsReenabledWithoutSpendingAnAttempt) {
	PairScenario s;
	s.run(100);
	s.drive.open = true;
	s.run(2000);
	s.drive.open = false;
	const auto out = s.run(2500);
	EXPECT_FALSE(s.drive.latched) << "the drive needs a disable, then an enable";
	EXPECT_EQ(s.machine.state(), kelo::WHEEL_NORMAL_OPERATION);
	EXPECT_TRUE(out.allowsEnable);
	EXPECT_FALSE(out.holdAtZero);
	EXPECT_EQ(s.machine.attempts(), 0u);
	EXPECT_TRUE(s.saw(kelo::RECOVERY_EVENT_SAFE_TORQUE_OFF_CLEARED));
}

TEST(WheelRecoveryMachine, aClosedPairIsRetestedEvenWhenOssdOkStaysClearUntilTheReenable) {
	PairScenario s;
	s.drive.ossdOkLatched = true;
	s.run(100);
	s.drive.open = true;
	s.run(3000);
	s.drive.open = false;
	s.run(6000);
	EXPECT_FALSE(s.drive.latched);
	EXPECT_EQ(s.machine.state(), kelo::WHEEL_NORMAL_OPERATION);
	EXPECT_EQ(s.machine.attempts(), 0u);
}

TEST(WheelRecoveryMachine, repeatedEmergencyStopsNeverAddUpToAGiveUp) {
	PairScenario s;
	double t = 100;
	s.run(t);
	for (int press = 0; press < 20; press++) {
		s.drive.open = true;
		s.run(t += 1000);
		s.drive.open = false;
		s.run(t += 1000);
	}
	EXPECT_FALSE(s.machine.failed());
	EXPECT_EQ(s.machine.state(), kelo::WHEEL_NORMAL_OPERATION);
	EXPECT_EQ(s.machine.attempts(), 0u);
}

TEST(WheelRecoveryMachine, anOperatorDisableEndsSafeTorqueOff) {
	PairScenario s;
	s.drive.open = true;
	s.run(500);
	ASSERT_EQ(s.machine.state(), kelo::WHEEL_SAFE_TORQUE_OFF);
	const auto out = s.run(1500, false);
	EXPECT_EQ(s.machine.state(), kelo::WHEEL_NORMAL_OPERATION);
	EXPECT_TRUE(s.saw(kelo::RECOVERY_EVENT_ABORTED_BY_OPERATOR));
	EXPECT_FALSE(out.holdAtZero);
}

TEST(WheelRecoveryMachine, aSlaveThatReturnsInSafeTorqueOffIsHeldInsteadOfGivenUp) {
	Scenario s;
	s.run(100, true, false);
	s.run(1100, false, true, false);
	const auto out = s.run(12000, true, true, false, true, true);
	EXPECT_TRUE(s.saw(kelo::RECOVERY_EVENT_SAFE_TORQUE_OFF));
	EXPECT_FALSE(s.machine.failed());
	EXPECT_TRUE(out.holdAtZero);
}

TEST(WheelRecoveryMachine, aZeroedImageDuringSafeTorqueOffIsNotAClosedPair) {
	Scenario s;
	s.run(100, true, false);
	s.run(1000, true, true, false, true, true);
	// The slave's image is zeroed before the link monitor calls it lost.
	s.run(1400, true, true, false, true, false, false);
	EXPECT_EQ(s.machine.attempts(), 0u);
	EXPECT_FALSE(s.saw(kelo::RECOVERY_EVENT_SAFE_TORQUE_OFF_CLEARED));
	EXPECT_NE(s.machine.state(), kelo::WHEEL_NORMAL_OPERATION);
	s.run(3000, true, true, false, true, true);
	EXPECT_EQ(s.machine.attempts(), 0u);
}

TEST(WheelRecoveryMachine, aLinkLossDuringSafeTorqueOffIsWaitedForAndReturnsToIt) {
	Scenario s;
	s.run(100, true, false);
	s.run(500, true, true, false, true, true);
	s.run(2500, false, true, false, true, false, false);
	EXPECT_EQ(s.machine.state(), kelo::WHEEL_LINK_LOST);
	s.run(4000, true, true, false, true, true);
	EXPECT_FALSE(s.machine.failed());
	EXPECT_EQ(s.machine.attempts(), 0u);
}

TEST(WheelRecoveryMachine, aGlitchRightAfterSafeTorqueOffClearedStillWaitsOutTheLatch) {
	PairScenario s;
	s.run(100);
	s.drive.open = true;
	s.run(2000);
	s.drive.open = false;
	while (!s.saw(kelo::RECOVERY_EVENT_SAFE_TORQUE_OFF_CLEARED))
		s.run(s.nowMs + 1);
	// One faulty-looking frame is shorter than the latch: no attempt.
	s.machine.step({s.nowMs + 1, true, true, true, false, false, true});
	s.nowMs += 1;
	s.run(s.nowMs + 50);
	EXPECT_EQ(s.machine.attempts(), 0u);
}

TEST(WheelRecoveryMachine, safeTorqueOffNeverRevivesAFailedWheel) {
	Scenario s;
	s.run(3000, true, true, false);
	ASSERT_TRUE(s.machine.failed());
	const auto out = s.run(4000, true, true, false, true, true);
	EXPECT_TRUE(s.machine.failed());
	EXPECT_FALSE(out.allowsEnable);
}

TEST(WheelStatusSane, acceptsTheValuesSeenOnAHealthyOrReturningDrive) {
	EXPECT_TRUE(kelo::wheelStatusSane(63, 2051));
	EXPECT_TRUE(kelo::wheelStatusSane(61, 2051));
	// MOTOR_STOP with the EtherCAT watchdog latched: every slave back from a
	// dropout reports it, and the disable-enable clears it.
	EXPECT_TRUE(kelo::wheelStatusSane(4157, 2051));
}

TEST(WheelStatusSane, rejectsFaultBitsAndAnUnresponsiveDrive) {
	EXPECT_FALSE(kelo::wheelStatusSane(0, 2051));
	// 125 (LOW_VOLTAGE_ERR) was seen when a wheel's supply collapsed.
	EXPECT_FALSE(kelo::wheelStatusSane(125, 2051));
	for (const std::uint16_t bit : {0x0040, 0x0080, 0x0100, 0x0200, 0x0400})
		EXPECT_FALSE(kelo::wheelStatusSane(static_cast<std::uint16_t>(63 | bit), 2051)) << std::hex << bit;
}

TEST(WheelStatusSane, rejectsTheSafetyNetAndEncoderErrors) {
	// OVER_SPEED_ERR, M2_ENC_ERR, M1_ENC_ERR, RED_ENC_ERR.
	for (const std::uint16_t bit : {0x0800, 0x2000, 0x4000, 0x8000})
		EXPECT_FALSE(kelo::wheelStatusSane(static_cast<std::uint16_t>(63 | bit), 2051)) << std::hex << bit;
	// 32829 (RED_ENC_ERR) followed a supply collapse.
	EXPECT_FALSE(kelo::wheelStatusSane(32829, 2051));
}

TEST(WheelStatusSane, rejectsAClearedOkFlag) {
	// OSSD_OK, PS_OK.
	EXPECT_FALSE(kelo::wheelStatusSane(63 & ~0x0004, 2051));
	EXPECT_FALSE(kelo::wheelStatusSane(63 & ~0x0008, 2051));
}

TEST(WheelStatusSane, rejectsStatus2Errors) {
	// Over/under voltage and current, board and motor temperatures, power
	// stages, OSSD channels: every STATUS2 error bit.
	for (int b = 2; b < 16; b++) {
		if (b == 11)
			continue;  // INT_SENSOR_OK
		const auto bit = static_cast<std::uint16_t>(1u << b);
		EXPECT_FALSE(kelo::wheelStatusSane(63, static_cast<std::uint16_t>(2051 | bit))) << "B" << b;
	}
	// 18435: OSSD1_ERR, seen at start (with status1 1085, EXT_DISABLE_ERR).
	EXPECT_FALSE(kelo::wheelStatusSane(63, 18435));
}

TEST(WheelRecoveryMachine, aHealthyWheelStaysNormalAndEnabled) {
	Scenario s;
	const auto out = s.run(1000, true, false);
	EXPECT_EQ(s.machine.state(), kelo::WHEEL_NORMAL_OPERATION);
	EXPECT_TRUE(out.allowsEnable);
	EXPECT_FALSE(out.holdAtZero);
	EXPECT_EQ(s.events, 0u);
}

TEST(WheelRecoveryMachine, aFaultThatClearsAfterTheReenableIsRecoveredOnce) {
	Scenario s;
	s.run(100, true, false);
	auto out = s.run(125, true, true);  // fault latched for 25 ms
	EXPECT_TRUE(s.saw(kelo::RECOVERY_EVENT_STARTED));
	EXPECT_EQ(s.machine.attempts(), 1u);
	EXPECT_TRUE(out.holdAtZero);
	s.run(140, true, true);
	out = s.run(200, true, false);  // drive healthy again
	EXPECT_TRUE(s.saw(kelo::RECOVERY_EVENT_SUCCEEDED));
	EXPECT_EQ(s.machine.state(), kelo::WHEEL_NORMAL_OPERATION);
	EXPECT_TRUE(out.allowsEnable);
	EXPECT_FALSE(out.holdAtZero);
}

TEST(WheelRecoveryMachine, disablesTheWheelBeforeReenablingIt) {
	Scenario s;
	s.run(15, true, true);
	// Inside the disable window the enable is withheld.
	const auto out = s.run(20, true, true);
	EXPECT_EQ(s.machine.state(), kelo::WHEEL_STATUS_RECOVERY_SENDING_DISABLE);
	EXPECT_FALSE(out.allowsEnable);
}

TEST(WheelRecoveryMachine, aFaultThatNeverClearsGivesUpAfterBoundedRetries) {
	Scenario s;
	s.run(50, true, false);
	// Fault bits present: not sane, so the link-loss allowance does not apply.
	s.run(3000, true, true, false);
	EXPECT_TRUE(s.machine.failed());
	EXPECT_TRUE(s.saw(kelo::RECOVERY_EVENT_GAVE_UP));
	EXPECT_EQ(s.machine.attempts(), 11u);
}

TEST(WheelRecoveryMachine, failureIsTerminalAndKeepsTheWheelDisabled) {
	Scenario s;
	s.run(3000, true, true, false);
	ASSERT_TRUE(s.machine.failed());
	const auto out = s.run(4000, true, false);
	EXPECT_TRUE(s.machine.failed());
	EXPECT_FALSE(out.allowsEnable);
	EXPECT_TRUE(out.holdAtZero);
}

TEST(WheelRecoveryMachine, aLinkLossDoesNotSpendRecoveryAttempts) {
	Scenario s;
	s.run(100, true, false);
	// 1.9 s with the slave gone: inputs zeroed, so status looks faulty too.
	const auto out = s.run(2000, false, true, false);
	EXPECT_EQ(s.machine.state(), kelo::WHEEL_LINK_LOST);
	EXPECT_EQ(s.machine.attempts(), 0u);
	EXPECT_FALSE(out.allowsEnable);
	EXPECT_TRUE(out.holdAtZero);
	EXPECT_TRUE(s.saw(kelo::RECOVERY_EVENT_LINK_LOST));
	EXPECT_FALSE(s.machine.failed());
}

TEST(WheelRecoveryMachine, aSlaveThatReturnsSaneIsReenabledAfterTheStableWindow) {
	Scenario s;
	s.run(100, true, false);
	s.run(2000, false, true, false);
	// Back, disabled (status 61): needs recovery but is sane.
	s.run(2050, true, true, true);
	EXPECT_EQ(s.machine.state(), kelo::WHEEL_LINK_LOST) << "must not re-enable before the link is stable";
	s.run(2200, true, true, true);
	EXPECT_TRUE(s.saw(kelo::RECOVERY_EVENT_STARTED));
	EXPECT_EQ(s.machine.attempts(), 1u);
	s.run(2300, true, false);
	EXPECT_EQ(s.machine.state(), kelo::WHEEL_NORMAL_OPERATION);
	EXPECT_FALSE(s.machine.failed());
}

TEST(WheelRecoveryMachine, aLinkLossLongerThanTheWindowGivesUp) {
	Scenario s;
	s.run(100, true, false);
	s.run(100 + 5000 - 10, false, true, false);
	EXPECT_FALSE(s.machine.failed());
	s.run(100 + 5000 + 50, false, true, false);
	EXPECT_TRUE(s.machine.failed());
	EXPECT_TRUE(s.saw(kelo::RECOVERY_EVENT_LINK_LOSS_TIMEOUT));
}

TEST(WheelRecoveryMachine, aSlaveThatReturnsWithFaultBitsIsNeverReenabled) {
	Scenario s;
	s.run(100, true, false);
	s.run(500, false, true, false);
	// Back with a supply-fault type bit that never clears.
	bool everAllowed = false;
	while (s.nowMs < 500 + 6000 && !s.machine.failed()) {
		const auto out = s.run(s.nowMs + 1, true, true, false);
		everAllowed = everAllowed || out.allowsEnable;
	}
	EXPECT_FALSE(everAllowed);
	EXPECT_TRUE(s.machine.failed());
	EXPECT_TRUE(s.saw(kelo::RECOVERY_EVENT_LINK_LOSS_TIMEOUT));
}

TEST(WheelRecoveryMachine, aFlappingSlaveWithFaultBitsCannotHoldThePlatformForever) {
	Scenario s;
	s.run(100, true, false);
	double t = 100;
	while (t < 100 + 5300 && !s.machine.failed()) {
		s.run(t + 30, false, true, false);
		s.run(t + 50, true, true, false);
		t += 50;
	}
	EXPECT_TRUE(s.machine.failed());
}

TEST(WheelRecoveryMachine, cleanReturnsAreCountedSoRepeatedDropoutsGiveUp) {
	Scenario s;
	double t = 100;
	s.run(t, true, false);
	int dropouts = 0;
	while (!s.machine.failed() && dropouts < 30) {
		s.run(t + 200, false, true, false);
		s.run(t + 400, true, false);  // back and already healthy
		t += 400;
		dropouts++;
	}
	EXPECT_TRUE(s.machine.failed());
	EXPECT_LE(dropouts, 12);
}

TEST(WheelRecoveryMachine, aMachineStartedInTheMiddleOfARunWaitsOutTheLatch) {
	WheelRecoveryMachine machine(WheelRecoveryConfig(), 90000.0);
	const auto out = machine.step({90001.0, true, true, true, true});
	EXPECT_EQ(machine.state(), kelo::WHEEL_NORMAL_OPERATION);
	EXPECT_EQ(out.events, 0u);
}

TEST(WheelRecoveryMachine, aFlappingLinkCannotResetTheLossWindow) {
	Scenario s;
	s.run(100, true, false);
	double t = 100;
	while (t < 100 + 5200 && !s.machine.failed()) {
		s.run(t + 30, false, true, false);
		s.run(t + 50, true, true, true);  // up for 20 ms, shorter than the stable window
		t += 50;
	}
	EXPECT_TRUE(s.machine.failed());
	EXPECT_TRUE(s.saw(kelo::RECOVERY_EVENT_LINK_LOSS_TIMEOUT));
	EXPECT_LE(t, 100 + 5200);
}

TEST(WheelRecoveryMachine, repeatedDropoutsInsideTheCounterWindowEventuallyGiveUp) {
	Scenario s;
	double t = 100;
	s.run(t, true, false);
	int dropouts = 0;
	while (!s.machine.failed() && dropouts < 30) {
		s.run(t + 200, false, true, false);       // gone for 200 ms
		s.run(t + 400, true, true, true);         // back, disabled
		s.run(t + 500, true, false);              // recovered
		t += 500;
		dropouts++;
	}
	EXPECT_TRUE(s.machine.failed());
	EXPECT_LE(dropouts, 12);
	EXPECT_TRUE(s.saw(kelo::RECOVERY_EVENT_GAVE_UP));
}

TEST(WheelRecoveryMachine, anOperatorDisableAbortsARecoveryInProgress) {
	Scenario s;
	s.run(100, true, false);
	s.run(2000, false, true, false);
	ASSERT_EQ(s.machine.state(), kelo::WHEEL_LINK_LOST);
	const auto out = s.run(2100, false, true, false, false);
	EXPECT_EQ(s.machine.state(), kelo::WHEEL_NORMAL_OPERATION);
	EXPECT_TRUE(s.saw(kelo::RECOVERY_EVENT_ABORTED_BY_OPERATOR));
	EXPECT_FALSE(out.holdAtZero);
}

TEST(WheelRecoveryMachine, aWheelTheOperatorDisabledIsNotRecoveredWhenItsLinkDrops) {
	Scenario s;
	s.run(100, true, false, true, false);
	s.run(3000, false, false, false, false);
	EXPECT_EQ(s.machine.state(), kelo::WHEEL_NORMAL_OPERATION);
	EXPECT_EQ(s.events, 0u);
}

TEST(WheelRecoveryMachine, attemptsResetAfterALongStretchOfNormalOperation) {
	Scenario s;
	s.run(50, true, true);
	s.run(200, true, false);
	ASSERT_EQ(s.machine.attempts(), 1u);
	s.run(200 + 31000, true, false);
	EXPECT_EQ(s.machine.attempts(), 0u);
	EXPECT_TRUE(s.saw(kelo::RECOVERY_EVENT_ATTEMPTS_RESET));
}

TEST(SetpointVectorSlew, followsTheTargetExactlyWhenNothingHolds) {
	kelo::SetpointVectorSlew slew(2);
	float target[2] = {5.0f, -3.0f};
	float out[2];
	slew.step(target, false, 0.1f, 0.1f, out);
	EXPECT_FLOAT_EQ(out[0], 5.0f);
	EXPECT_FLOAT_EQ(out[1], -3.0f);
}

TEST(SetpointVectorSlew, aHoldRampsToZeroInsteadOfCuttingIt) {
	kelo::SetpointVectorSlew slew(1);
	float target[1] = {1.0f};
	float out[1];
	slew.step(target, false, 0.25f, 0.25f, out);
	slew.step(target, true, 0.25f, 0.25f, out);
	EXPECT_FLOAT_EQ(out[0], 0.75f);
	for (int i = 0; i < 3; i++)
		slew.step(target, true, 0.25f, 0.25f, out);
	EXPECT_FLOAT_EQ(out[0], 0.0f);
}

TEST(SetpointVectorSlew, everySetpointReachesZeroInTheSameCycle) {
	kelo::SetpointVectorSlew slew(3);
	float target[3] = {4.0f, -1.0f, 0.5f};
	float out[3];
	slew.step(target, false, 0.5f, 0.5f, out);
	int cycles = 0;
	while ((out[0] != 0.0f || out[1] != 0.0f || out[2] != 0.0f) && cycles < 100) {
		slew.step(target, true, 0.5f, 0.5f, out);
		cycles++;
		// The fastest setpoint moves by the step, the others proportionally.
		const float ratio = out[1] / out[0];
		if (out[0] != 0.0f)
			EXPECT_NEAR(ratio, -0.25f, 1e-4f);
	}
	EXPECT_EQ(cycles, 8);
	EXPECT_FLOAT_EQ(out[1], 0.0f);
	EXPECT_FLOAT_EQ(out[2], 0.0f);
}

TEST(SetpointVectorSlew, afterTheHoldItRampsBackUpAtTheUpStep) {
	kelo::SetpointVectorSlew slew(1);
	float target[1] = {1.0f};
	float out[1];
	slew.step(target, false, 0.25f, 0.1f, out);
	for (int i = 0; i < 4; i++)
		slew.step(target, true, 0.25f, 0.1f, out);
	ASSERT_FLOAT_EQ(out[0], 0.0f);
	slew.step(target, false, 0.25f, 0.1f, out);
	EXPECT_FLOAT_EQ(out[0], 0.1f) << "must not jump back to the live target";
	for (int i = 0; i < 9; i++)
		slew.step(target, false, 0.25f, 0.1f, out);
	EXPECT_FLOAT_EQ(out[0], 1.0f);
	target[0] = 0.4f;
	slew.step(target, false, 0.25f, 0.1f, out);
	EXPECT_FLOAT_EQ(out[0], 0.4f);
}

TEST(SetpointVectorSlew, resetForgetsTheRampAndTakesTheTarget) {
	kelo::SetpointVectorSlew slew(1);
	float target[1] = {1.0f};
	float out[1];
	slew.step(target, false, 0.25f, 0.25f, out);
	slew.step(target, true, 0.25f, 0.25f, out);
	slew.reset();
	EXPECT_FLOAT_EQ(slew.last()[0], 0.0f);
	target[0] = 2.0f;
	slew.step(target, false, 0.25f, 0.25f, out);
	EXPECT_FLOAT_EQ(out[0], 2.0f);
}

TEST(SetpointVectorSlew, aNonFiniteTargetIsTreatedAsZeroAndNeverStored) {
	kelo::SetpointVectorSlew slew(2);
	float target[2] = {std::nanf(""), 2.0f};
	float out[2];
	slew.step(target, false, 0.5f, 0.5f, out);
	EXPECT_TRUE(std::isfinite(out[0]));
	EXPECT_FLOAT_EQ(out[0], 0.0f);
	EXPECT_TRUE(std::isfinite(slew.last()[0]));
	target[0] = 1.0f;
	slew.step(target, false, 0.5f, 0.5f, out);
	EXPECT_TRUE(std::isfinite(out[0]));
	EXPECT_TRUE(std::isfinite(out[1]));
}

TEST(ReleaseStepPerCycle, usesTheGentlerOfTheShapingRateAndTheDefault) {
	EXPECT_FLOAT_EQ(kelo::releaseStepPerCycle(0.0f, 0.05f), 0.05f);
	EXPECT_FLOAT_EQ(kelo::releaseStepPerCycle(10.0f, 0.05f), 0.01f);
	EXPECT_FLOAT_EQ(kelo::releaseStepPerCycle(200.0f, 0.05f), 0.05f);
}

TEST(WheelRecoveryMachine, aSlaveThatReturnsJustBeforeTheWindowEndsGetsToFinishItsStabilityCheck) {
	Scenario s;
	s.run(100, true, false);
	s.run(100 + 4950, false, true, false);
	s.run(100 + 5200, true, true, true);
	EXPECT_FALSE(s.machine.failed());
	EXPECT_TRUE(s.saw(kelo::RECOVERY_EVENT_STARTED));
}

TEST(LinkMonitor, aSlaveInOperationalStateIsUp) {
	kelo::LinkMonitor monitor;
	EXPECT_TRUE(monitor.update(0, false, true));
	EXPECT_TRUE(monitor.update(1000, false, true));
}

TEST(LinkMonitor, aSlaveTheCheckThreadDeclaredLostIsDownAtOnce) {
	kelo::LinkMonitor monitor;
	EXPECT_FALSE(monitor.update(0, true, true));
	EXPECT_FALSE(monitor.update(1, true, false));
}

TEST(LinkMonitor, aBriefNonOperationalStateFromALostFrameDoesNotCount) {
	kelo::LinkMonitor monitor;
	EXPECT_TRUE(monitor.update(0, false, true));
	EXPECT_TRUE(monitor.update(10, false, false));
	EXPECT_TRUE(monitor.update(70, false, false));
	EXPECT_TRUE(monitor.update(80, false, true));
	// The next lost frame starts a fresh window.
	EXPECT_TRUE(monitor.update(90, false, false));
	EXPECT_TRUE(monitor.update(300, false, false));
}

TEST(LinkMonitor, aLastingNonOperationalStateIsDownUntilOperationalAgain) {
	kelo::LinkMonitor monitor;
	EXPECT_TRUE(monitor.update(0, false, false));
	EXPECT_TRUE(monitor.update(250, false, false));
	EXPECT_FALSE(monitor.update(251, false, false));
	EXPECT_FALSE(monitor.update(2000, false, false));
	EXPECT_TRUE(monitor.update(2001, false, true));
}

}  // namespace
