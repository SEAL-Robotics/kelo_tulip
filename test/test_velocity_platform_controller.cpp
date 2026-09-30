#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "kelo_tulip/VelocityPlatformController.h"

using namespace kelo;

namespace {

constexpr float DT = 0.001f;  // the EtherCAT period

std::vector<WheelConfig> fourWheels() {
	std::vector<WheelConfig> wheels(4);
	const double xs[4] = {1.0, 1.0, -1.0, -1.0};
	const double ys[4] = {-0.25, 0.25, 0.25, -0.25};
	for (int i = 0; i < 4; i++) {
		wheels[i].x = xs[i];
		wheels[i].y = ys[i];
		wheels[i].a = 0.0;
		wheels[i].reverseVelocity = false;
		wheels[i].model.diameter = 0.165;
		wheels[i].model.casteroffset = 0.015;
		wheels[i].model.wheeldistance = 0.184;
		wheels[i].model.velocitylimit = 100.0;
	}
	return wheels;
}

// One ramping step with limits wide open, so the ramped velocity is the target.
void driveAt(VelocityPlatformController &c, float vx, float vy, float dt = DT) {
	c.setPlatformMaxLinVelocity(10.0f);
	c.setPlatformMaxAngVelocity(10.0f);
	c.setPlatformMaxLinAcceleration(1e6f);
	c.setPlatformMaxAngAcceleration(1e6f);
	c.setPlatformMaxLinDeceleration(1e6f);
	c.setPlatformMaxAngDeceleration(1e6f);
	c.setPlatformTargetVelocity(vx, vy, 0.0f);
	c.calculatePlatformRampedVelocities(dt);
}

void wheelSetpoints(VelocityPlatformController &c, float pivot, float out[4][2]) {
	for (size_t i = 0; i < 4; i++)
		c.calculateWheelTargetVelocity(i, pivot, out[i][0], out[i][1]);
}

// The pre-shaping calculateWheelTargetVelocity, copied verbatim apart from
// taking its inputs as arguments.
void legacyTarget(const WheelConfig &wc, float vx, float vy, float va, float raw_pivot_angle,
                         float &target_ang_vel_l, float &target_ang_vel_r) {
	if (vx == 0 && vy == 0 && va == 0) {
		target_ang_vel_l = 0.0f;
		target_ang_vel_r = 0.0f;
		return;
	}
	const float angular_to_linear_velocity = 0.5 * wc.model.diameter;
	const float linear_to_angular_velocity = 1.0 / angular_to_linear_velocity;
	const float max_linear_velocity = wc.model.velocitylimit * angular_to_linear_velocity;
	const float pivot_kp = 0.2f;
	const float max_pivot_error = M_PI * 0.25f;
	Point2D rel_l, rel_r;
	rel_l.x = -1 * wc.model.casteroffset;
	rel_l.y = 0.5 * wc.model.wheeldistance;
	rel_r.x = -1 * wc.model.casteroffset;
	rel_r.y = -0.5 * wc.model.wheeldistance;
	float pivot_angle = Utils::clipAngle(raw_pivot_angle - wc.a);
	Point2D u;
	u.x = cos(pivot_angle);
	u.y = sin(pivot_angle);
	Point2D pl, pr;
	pl.x = (rel_l.x * u.x - rel_l.y * u.y) + wc.x;
	pl.y = (rel_l.x * u.y + rel_l.y * u.x) + wc.y;
	pr.x = (rel_r.x * u.x - rel_r.y * u.y) + wc.x;
	pr.y = (rel_r.x * u.y + rel_r.y * u.x) + wc.y;
	Point2D tp;
	tp.x = vx - (va * wc.y);
	tp.y = vy + (va * wc.x);
	float target_pivot_angle = atan2(tp.y, tp.x);
	float pivot_error = Utils::getShortestAngle(target_pivot_angle, pivot_angle);
	pivot_error = Utils::clip(pivot_error, max_pivot_error, -max_pivot_error);
	Point2D tl, tr;
	tl.x = vx - (va * pl.y);
	tl.y = vy + (va * pl.x);
	tr.x = vx - (va * pr.y);
	tr.y = vy + (va * pr.x);
	float delta_vel = pivot_error * pivot_kp;
	float vel_l = tl.x * u.x + tl.y * u.y;
	if (wc.reverseVelocity) vel_l *= -1;
	float target_vel_l = Utils::clip(vel_l + delta_vel, max_linear_velocity, -max_linear_velocity);
	float vel_r = tr.x * u.x + tr.y * u.y;
	if (wc.reverseVelocity) vel_r *= -1;
	float target_vel_r = Utils::clip(vel_r - delta_vel, max_linear_velocity, -max_linear_velocity);
	target_ang_vel_l = target_vel_l * linear_to_angular_velocity;
	target_ang_vel_r = target_vel_r * linear_to_angular_velocity;
}

}  // namespace

TEST(VelocityPlatformControllerShaping, DefaultsAreBitIdenticalToLegacy) {
	std::mt19937 rng(7);
	auto uni = [&rng](float lo, float hi) { return std::uniform_real_distribution<float>(lo, hi)(rng); };
	auto wheels = fourWheels();
	wheels[1].reverseVelocity = true;
	VelocityPlatformController c;
	c.initialise(wheels);
	for (int trial = 0; trial < 200; trial++) {
		const float vx = uni(-1.5f, 1.5f), vy = uni(-1.5f, 1.5f);
		driveAt(c, vx, vy);
		for (size_t i = 0; i < 4; i++) {
			const float pivot = uni(-3.14f, 3.14f);
			float l, r, ll, lr;
			c.calculateWheelTargetVelocity(i, pivot, l, r);
			legacyTarget(wheels[i], vx, vy, 0.0f, pivot, ll, lr);
			ASSERT_EQ(l, ll);
			ASSERT_EQ(r, lr);
		}
	}
}

TEST(VelocityPlatformControllerShaping, SlewLimitsGrowthToTheAllowedStep) {
	VelocityPlatformController c;
	c.initialise(fourWheels());
	CurrentShapingConfig cfg;
	cfg.slewRateRadPerSecSq = 10.0f;
	c.setCurrentShaping(cfg);
	driveAt(c, 1.0f, 0.0f, 0.02f);  // dt is clamped to SLEW_MAX_DT_SEC for the slew
	const float step = 10.0f * SLEW_MAX_DT_SEC;
	float out[4][2];
	wheelSetpoints(c, 0.0f, out);
	for (int i = 0; i < 4; i++) {
		EXPECT_NEAR(out[i][0], step, 1e-6f);
		EXPECT_NEAR(out[i][1], step, 1e-6f);
	}
	wheelSetpoints(c, 0.0f, out);
	for (int i = 0; i < 4; i++) {
		EXPECT_NEAR(out[i][0], 2 * step, 1e-6f);
		EXPECT_NEAR(out[i][1], 2 * step, 1e-6f);
	}
}

TEST(VelocityPlatformControllerShaping, ReorientFirstScalesDownTranslation) {
	VelocityPlatformController plain, shaped;
	plain.initialise(fourWheels());
	shaped.initialise(fourWheels());
	CurrentShapingConfig cfg;
	cfg.reorientStartError = 0.3f;
	cfg.reorientFullError = 1.0f;
	cfg.reorientMinScale = 0.25f;
	shaped.setCurrentShaping(cfg);
	driveAt(plain, 0.5f, 0.0f);
	driveAt(shaped, 0.5f, 0.0f);
	float a[4][2], b[4][2];
	for (int pass = 0; pass < 2; pass++) {  // scale uses the previous pass
		wheelSetpoints(plain, 2.0f, a);
		wheelSetpoints(shaped, 2.0f, b);
	}
	for (int i = 0; i < 4; i++) {
		EXPECT_LE(std::fabs(b[i][0]), std::fabs(a[i][0]) + 1e-4f);
		EXPECT_LE(std::fabs(b[i][1]), std::fabs(a[i][1]) + 1e-4f);
	}
}

TEST(VelocityPlatformControllerShaping, ZeroCommandStopsImmediatelyEvenWithSlew) {
	VelocityPlatformController c;
	c.initialise(fourWheels());
	CurrentShapingConfig cfg;
	cfg.slewRateRadPerSecSq = 10.0f;
	c.setCurrentShaping(cfg);
	driveAt(c, 0.0f, 0.0f);
	float l = 1.0f, r = 1.0f;
	c.calculateWheelTargetVelocity(0, 0.0f, l, r);
	EXPECT_EQ(l, 0.0f);
	EXPECT_EQ(r, 0.0f);
}

TEST(VelocityPlatformControllerShaping, DisabledWheelIsCommandedZeroAndRestartsSlewFromZero) {
	VelocityPlatformController c;
	c.initialise(fourWheels());
	CurrentShapingConfig cfg;
	cfg.slewRateRadPerSecSq = 100.0f;
	c.setCurrentShaping(cfg);
	driveAt(c, 1.0f, 0.0f);
	float l, r;
	for (int i = 0; i < 20; i++)
		c.calculateWheelTargetVelocity(0, 0.0f, l, r);  // slew state builds up
	EXPECT_GT(l, 100.0f * DT * 5);
	c.setWheelActive(0, false);
	c.calculateWheelTargetVelocity(0, 0.0f, l, r);
	EXPECT_EQ(l, 0.0f);
	EXPECT_EQ(r, 0.0f);
	c.setWheelActive(0, true);  // as after a recovery re-enable
	c.calculateWheelTargetVelocity(0, 0.0f, l, r);
	EXPECT_NEAR(l, 100.0f * DT, 1e-4f);
	EXPECT_NEAR(r, 100.0f * DT, 1e-4f);
}

TEST(VelocityPlatformControllerShaping, SlewStepIsBoundedAfterALongPause) {
	VelocityPlatformController c;
	c.initialise(fourWheels());
	CurrentShapingConfig cfg;
	cfg.slewRateRadPerSecSq = 100.0f;
	c.setCurrentShaping(cfg);
	driveAt(c, 1.0f, 0.0f);
	float l, r;
	for (int i = 0; i < 20; i++)
		c.calculateWheelTargetVelocity(0, 0.0f, l, r);
	driveAt(c, 1.0f, 0.0f, 0.2f);  // the ramping step spans a 200 ms pause
	c.calculateWheelTargetVelocity(0, 0.0f, l, r);
	// state was reset by the pause and the step is clamped: not 200 ms of growth
	EXPECT_NEAR(l, 100.0f * SLEW_MAX_DT_SEC, 1e-4f);
	EXPECT_NEAR(r, 100.0f * SLEW_MAX_DT_SEC, 1e-4f);
}

TEST(VelocityPlatformControllerShaping, GapUnderTheSetpointTimeoutKeepsSlewState) {
	// hubs keep driving for 100 ms after the last setpoint, so a gap shorter
	// than that is a hiccup, not a pause
	VelocityPlatformController c;
	c.initialise(fourWheels());
	CurrentShapingConfig cfg;
	cfg.slewRateRadPerSecSq = 100.0f;
	c.setCurrentShaping(cfg);
	driveAt(c, 1.0f, 0.0f);
	float l, r;
	for (int i = 0; i < 20; i++)
		c.calculateWheelTargetVelocity(0, 0.0f, l, r);
	const float before = l;
	driveAt(c, 1.0f, 0.0f, 0.1f);
	c.calculateWheelTargetVelocity(0, 0.0f, l, r);
	EXPECT_NEAR(l, before + 100.0f * SLEW_MAX_DT_SEC, 1e-4f);
}

TEST(VelocityPlatformControllerShaping, DisabledWheelDoesNotThrottleOthers) {
	VelocityPlatformController c, plain;
	c.initialise(fourWheels());
	plain.initialise(fourWheels());
	CurrentShapingConfig cfg;
	cfg.reorientStartError = 0.3f;
	cfg.reorientFullError = 1.0f;
	cfg.reorientMinScale = 0.25f;
	c.setCurrentShaping(cfg);
	c.setWheelActive(0, false);
	driveAt(c, 0.5f, 0.0f);
	driveAt(plain, 0.5f, 0.0f);
	float out[4][2], ref[4][2];
	for (int pass = 0; pass < 2; pass++) {
		c.calculateWheelTargetVelocity(0, 2.5f, out[0][0], out[0][1]);  // far off
		for (size_t i = 1; i < 4; i++)
			c.calculateWheelTargetVelocity(i, 0.0f, out[i][0], out[i][1]);
	}
	wheelSetpoints(plain, 0.0f, ref);
	EXPECT_EQ(out[1][0], ref[1][0]);
}

TEST(VelocityPlatformControllerShaping, InvalidShapingFallsBackToLegacy) {
	VelocityPlatformController c, plain;
	c.initialise(fourWheels());
	plain.initialise(fourWheels());
	CurrentShapingConfig bad;
	bad.slewRateRadPerSecSq = std::nanf("");
	bad.pivotKp = 5.0f;
	c.setCurrentShaping(bad);
	driveAt(c, 0.5f, 0.2f);
	driveAt(plain, 0.5f, 0.2f);
	float a[4][2], b[4][2];
	wheelSetpoints(c, 1.0f, a);
	wheelSetpoints(plain, 1.0f, b);
	for (int i = 0; i < 4; i++) {
		EXPECT_EQ(a[i][0], b[i][0]);
		EXPECT_EQ(a[i][1], b[i][1]);
	}
}

TEST(VelocityPlatformControllerShaping, MinScaleNearZeroIsRejectedSoAStuckCasterCannotStallTheBase) {
	VelocityPlatformController c, plain;
	c.initialise(fourWheels());
	plain.initialise(fourWheels());
	CurrentShapingConfig cfg;
	cfg.reorientStartError = 0.3f;
	cfg.reorientFullError = 1.0f;
	cfg.reorientMinScale = 0.01f;  // would freeze translation: invalid
	ASSERT_FALSE(isValid(cfg));
	c.setCurrentShaping(cfg);
	driveAt(c, 0.5f, 0.0f);
	driveAt(plain, 0.5f, 0.0f);
	float out[4][2], ref[4][2];
	for (int pass = 0; pass < 3; pass++) {
		c.calculateWheelTargetVelocity(0, 2.5f, out[0][0], out[0][1]);  // stuck
		for (size_t i = 1; i < 4; i++)
			c.calculateWheelTargetVelocity(i, 0.0f, out[i][0], out[i][1]);
	}
	wheelSetpoints(plain, 0.0f, ref);
	EXPECT_EQ(out[1][0], ref[1][0]);
}

TEST(VelocityPlatformControllerShaping, ValidMinScaleStillMovesTheBaseWithAStuckCaster) {
	VelocityPlatformController c, plain;
	c.initialise(fourWheels());
	plain.initialise(fourWheels());
	CurrentShapingConfig cfg;
	cfg.reorientStartError = 0.3f;
	cfg.reorientFullError = 1.0f;
	cfg.reorientMinScale = MIN_REORIENT_SCALE;
	ASSERT_TRUE(isValid(cfg));
	c.setCurrentShaping(cfg);
	driveAt(c, 0.5f, 0.0f);
	driveAt(plain, 0.5f, 0.0f);
	float out[4][2], ref[4][2];
	for (int pass = 0; pass < 3; pass++) {
		c.calculateWheelTargetVelocity(0, 2.5f, out[0][0], out[0][1]);
		for (size_t i = 1; i < 4; i++)
			c.calculateWheelTargetVelocity(i, 0.0f, out[i][0], out[i][1]);
	}
	wheelSetpoints(plain, 0.0f, ref);
	EXPECT_NEAR(out[1][0], MIN_REORIENT_SCALE * ref[1][0], 1e-3f);
	EXPECT_GT(std::fabs(out[1][0]), 0.0f);
}

TEST(VelocityPlatformControllerShaping, ShapedHubSetpointNeverExceedsLegacyProperty) {
	std::mt19937 rng(12345);
	auto uni = [&rng](float lo, float hi) { return std::uniform_real_distribution<float>(lo, hi)(rng); };
	for (int trial = 0; trial < 300; trial++) {
		auto wheels = fourWheels();
		for (auto &w : wheels) w.reverseVelocity = uni(0, 1) < 0.5f;
		VelocityPlatformController c;
		c.initialise(wheels);
		CurrentShapingConfig cfg;
		cfg.slewRateRadPerSecSq = uni(0, 1) < 0.5f ? 0.0f : uni(0.5f, 200.0f);
		cfg.pivotKp = uni(0.0f, 0.2f);
		cfg.maxPivotError = uni(0.05f, LEGACY_MAX_PIVOT_ERROR);
		cfg.maxPivotCorrectionSpeed = uni(0, 1) < 0.5f ? 0.0f : uni(0.001f, 0.3f);
		if (uni(0, 1) < 0.7f) {
			cfg.reorientStartError = uni(0.05f, 0.8f);
			cfg.reorientFullError = cfg.reorientStartError + uni(0.05f, 2.0f);
			cfg.reorientMinScale = uni(0.1f, 1.0f);
		}
		ASSERT_TRUE(isValid(cfg));
		c.setCurrentShaping(cfg);
		const float vx = uni(-1.5f, 1.5f), vy = uni(-1.5f, 1.5f), va = uni(-0.5f, 0.5f);
		c.setPlatformMaxLinVelocity(10.0f);
		c.setPlatformMaxAngVelocity(10.0f);
		c.setPlatformMaxLinAcceleration(1e6f);
		c.setPlatformMaxAngAcceleration(1e6f);
		c.setPlatformMaxLinDeceleration(1e6f);
		c.setPlatformMaxAngDeceleration(1e6f);
		c.setPlatformTargetVelocity(vx, vy, va);
		c.calculatePlatformRampedVelocities(DT);
		for (int step = 0; step < 30; step++) {
			for (size_t i = 0; i < 4; i++) {
				const float pivot = uni(-3.14f, 3.14f);
				float l, r, ll, lr;
				c.calculateWheelTargetVelocity(i, pivot, l, r);
				legacyTarget(wheels[i], vx, vy, va, pivot, ll, lr);
				ASSERT_LE(std::fabs(l), std::fabs(ll) + 1e-4f) << "trial " << trial;
				ASSERT_LE(std::fabs(r), std::fabs(lr) + 1e-4f) << "trial " << trial;
			}
		}
	}
}
