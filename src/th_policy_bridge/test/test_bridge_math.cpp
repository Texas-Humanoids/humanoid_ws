#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "th_policy_bridge/heading_hold.hpp"
#include "th_policy_bridge/joint_mapping.hpp"
#include "th_policy_bridge/observation.hpp"
#include "th_policy_bridge/safety.hpp"
#include "th_policy_bridge/stand_balance.hpp"

namespace tb = th_policy_bridge;

TEST(ProjectedGravity, UprightPointsDown)
{
  const auto g = tb::projected_gravity({1.0, 0.0, 0.0, 0.0});
  EXPECT_NEAR(g[0], 0.0, 1e-12);
  EXPECT_NEAR(g[1], 0.0, 1e-12);
  EXPECT_NEAR(g[2], -1.0, 1e-12);
}

TEST(ProjectedGravity, PitchedForwardTiltsGravityTowardBodyX)
{
  // +90 degrees about body y (nose down): gravity lies along body +x.
  const double h = std::sqrt(0.5);
  const auto g = tb::projected_gravity({h, 0.0, h, 0.0});
  EXPECT_NEAR(g[0], 1.0, 1e-12);
  EXPECT_NEAR(g[1], 0.0, 1e-12);
  EXPECT_NEAR(g[2], 0.0, 1e-12);
}

TEST(ProjectedGravity, RolledLeftAndUnnormalized)
{
  // +90 degrees about body x, scaled by 2: gravity lies along body -y.
  const double h = 2.0 * std::sqrt(0.5);
  const auto g = tb::projected_gravity({h, h, 0.0, 0.0});
  EXPECT_NEAR(g[0], 0.0, 1e-12);
  EXPECT_NEAR(g[1], -1.0, 1e-12);
  EXPECT_NEAR(g[2], 0.0, 1e-12);
}

TEST(ProjectedGravity, YawDoesNotChangeGravity)
{
  const auto g = tb::projected_gravity({std::cos(0.4), 0.0, 0.0, std::sin(0.4)});
  EXPECT_NEAR(g[2], -1.0, 1e-12);
}

TEST(ProjectedGravity, RejectsInvalidQuaternions)
{
  EXPECT_THROW(tb::projected_gravity({0.0, 0.0, 0.0, 0.0}), std::invalid_argument);
  EXPECT_THROW(
    tb::projected_gravity({std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0}),
    std::invalid_argument);
}

TEST(Actions, InvertsPolicyTargetScaling)
{
  const auto actions = tb::actions_from_targets({0.25, -0.2, 0.9}, {0.0, -0.2, 0.4}, 0.25);
  ASSERT_EQ(actions.size(), 3u);
  EXPECT_DOUBLE_EQ(actions[0], 1.0);
  EXPECT_DOUBLE_EQ(actions[1], 0.0);
  EXPECT_DOUBLE_EQ(actions[2], 2.0);
  EXPECT_THROW(tb::actions_from_targets({0.0}, {0.0, 0.0}, 0.25), std::invalid_argument);
  EXPECT_THROW(tb::actions_from_targets({0.0}, {0.0}, 0.0), std::invalid_argument);
}

TEST(Observation, FollowsBerkeleyOrder)
{
  tb::ObservationInputs in;
  in.command = {0.1, 0.2, 0.3};
  in.angular_velocity = {1.0, 2.0, 3.0};
  in.gravity = {0.0, 0.0, -1.0};
  in.joint_positions = {0.5, 0.4};
  in.joint_velocities = {-1.0, 1.0};
  in.previous_actions = {7.0, 8.0};
  const auto obs = tb::build_observation(in, {0.5, 0.1});
  const std::vector<float> expected{
    0.1F, 0.2F, 0.3F, 1.0F, 2.0F, 3.0F, 0.0F, 0.0F, -1.0F,
    0.0F, 0.3F, -1.0F, 1.0F, 7.0F, 8.0F};
  ASSERT_EQ(obs.size(), expected.size());
  for (size_t i = 0; i < obs.size(); ++i) {
    EXPECT_NEAR(obs[i], expected[i], 1e-6) << "index " << i;
  }
}

TEST(Observation, BerkeleyProfileHas75Values)
{
  tb::ObservationInputs in{};
  in.joint_positions.assign(22, 0.0);
  in.joint_velocities.assign(22, 0.0);
  in.previous_actions.assign(22, 0.0);
  EXPECT_EQ(tb::build_observation(in, std::vector<double>(22, 0.0)).size(), 75u);
}

TEST(Observation, RejectsWrongSizesAndNonFiniteValues)
{
  tb::ObservationInputs in{};
  in.joint_positions = {0.0};
  in.joint_velocities = {0.0};
  in.previous_actions = {0.0};
  EXPECT_THROW(tb::build_observation(in, {0.0, 0.0}), std::invalid_argument);
  in.joint_velocities = {std::numeric_limits<double>::infinity()};
  EXPECT_THROW(tb::build_observation(in, {0.0}), std::invalid_argument);
}

TEST(JointMapping, AppliesLimitsDirectionAndOffset)
{
  const tb::JointMotor m{-1.0, 0.5, -0.2, 1.0};
  EXPECT_DOUBLE_EQ(tb::joint_to_motor(m, 0.3), 0.2);
  EXPECT_DOUBLE_EQ(tb::joint_to_motor(m, 5.0), -0.5);  // clamped to upper = 1.0
  EXPECT_DOUBLE_EQ(tb::joint_to_motor(m, -5.0), 0.7);  // clamped to lower = -0.2
  EXPECT_DOUBLE_EQ(tb::motor_to_joint(m, 0.2), 0.3);
  EXPECT_THROW(
    tb::joint_to_motor(m, std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}

TEST(JointMapping, RejectsInvalidCalibration)
{
  EXPECT_NO_THROW(tb::validate({1.0, 0.0, -1.0, 1.0}, "j"));
  EXPECT_THROW(tb::validate({0.5, 0.0, -1.0, 1.0}, "j"), std::invalid_argument);
  EXPECT_THROW(tb::validate({1.0, 0.0, 1.0, 1.0}, "j"), std::invalid_argument);
  EXPECT_THROW(
    tb::validate({1.0, std::numeric_limits<double>::infinity(), -1.0, 1.0}, "j"),
    std::invalid_argument);
}

TEST(Rotation, QuarterTurnAboutZ)
{
  const double h = std::sqrt(0.5);
  const auto v = tb::rotate({h, 0.0, 0.0, h}, {1.0, 0.0, 0.0});
  EXPECT_NEAR(v[0], 0.0, 1e-12);
  EXPECT_NEAR(v[1], 1.0, 1e-12);
  EXPECT_NEAR(v[2], 0.0, 1e-12);
}

TEST(Gravity, NoseDownPitchGivesPositiveX)
{
  // Pitch +0.3 rad about y (nose down in a z-up frame).
  const double a = 0.15;
  const auto g = tb::projected_gravity({std::cos(a), 0.0, std::sin(a), 0.0});
  EXPECT_NEAR(g[0], std::sin(0.3), 1e-12);
  EXPECT_NEAR(g[2], -std::cos(0.3), 1e-12);
}

TEST(Gravity, NedWorldMatchesEnuForUprightImu)
{
  // Upright in NED means the IMU's own z points down; a 180 deg turn about x maps
  // that IMU into a z-up base frame.
  const auto in_imu = tb::projected_gravity({1.0, 0.0, 0.0, 0.0}, true);
  const auto in_base = tb::rotate({0.0, 1.0, 0.0, 0.0}, in_imu);
  EXPECT_NEAR(in_base[2], -1.0, 1e-12);
}

TEST(Layout, WalkerV2LayoutHas41ScaledEntries)
{
  const auto layout = tb::parse_layout(
    {"base_ang_vel", "projected_gravity", "velocity_command", "joint_pos_rel", "joint_vel",
      "last_action", "gait_phase"},
    {0.25, 1.0, 1.0, 1.0, 0.05, 1.0, 1.0});
  const std::vector<double> defaults(10, 0.1);
  tb::ObservationInputs in;
  in.command = {0.3, 0.0, 0.1};
  in.angular_velocity = {4.0, 0.0, 0.0};
  in.gravity = {0.0, 0.0, -1.0};
  in.joint_positions.assign(10, 0.3);
  in.joint_velocities.assign(10, 2.0);
  in.previous_actions.assign(10, -1.0);
  in.gait_phase = 0.25;
  const auto obs = tb::build_observation(in, defaults, layout);
  ASSERT_EQ(obs.size(), 41u);
  ASSERT_EQ(layout.size(10), 41u);
  EXPECT_FLOAT_EQ(obs[0], 1.0f);  // ang vel x 0.25
  EXPECT_FLOAT_EQ(obs[5], -1.0f);  // gravity z
  EXPECT_FLOAT_EQ(obs[6], 0.3f);  // command vx
  EXPECT_FLOAT_EQ(obs[9], 0.2f);  // joint pos rel
  EXPECT_FLOAT_EQ(obs[19], 0.1f);  // joint vel x 0.05
  EXPECT_FLOAT_EQ(obs[29], -1.0f);  // last action
  EXPECT_NEAR(obs[39], 1.0f, 1e-6);  // sin(pi/2)
  EXPECT_NEAR(obs[40], 0.0f, 1e-6);  // cos(pi/2)
}

TEST(Layout, EmptyNamesFallBackToBerkeley)
{
  EXPECT_EQ(tb::parse_layout({}, {}).size(12), 45u);
}

TEST(Layout, RejectsUnknownTermAndScaleMismatch)
{
  EXPECT_THROW(tb::parse_layout({"height_scan"}, {}), std::invalid_argument);
  EXPECT_THROW(tb::parse_layout({"joint_vel"}, {1.0, 2.0}), std::invalid_argument);
}

TEST(Interpolate, ClampsAlpha)
{
  const auto mid = tb::interpolate({0.0, 1.0}, {1.0, 3.0}, 0.5);
  EXPECT_DOUBLE_EQ(mid[0], 0.5);
  EXPECT_DOUBLE_EQ(mid[1], 2.0);
  EXPECT_DOUBLE_EQ(tb::interpolate({0.0}, {1.0}, 7.0)[0], 1.0);
}

namespace
{
const std::array<double, 3> kUpright{0.0, 0.0, -1.0};
const std::array<double, 3> kStill{0.0, 0.0, 0.0};
}  // namespace

TEST(HeadingHold, DisabledPassesCommandThrough)
{
  tb::HeadingHold hold(0.0, 0.3);
  const auto out = hold.update({0.3, 0.0, 0.0}, kUpright, {0.0, 0.0, 0.5}, 1.0);
  EXPECT_EQ(out, (std::array<double, 3>{0.3, 0.0, 0.0}));
}

TEST(HeadingHold, CorrectsYawDriftWhileWalkingStraight)
{
  tb::HeadingHold hold(1.5, 0.3);
  hold.update({0.3, 0.0, 0.0}, kUpright, kStill, 0.02);  // reference = 0
  // The robot yaws left (+z) by 0.1 rad: the correction turns it right.
  const auto out = hold.update({0.3, 0.0, 0.0}, kUpright, {0.0, 0.0, 0.1 / 0.02}, 0.02);
  EXPECT_NEAR(hold.heading(), 0.1, 1e-12);
  EXPECT_DOUBLE_EQ(out[0], 0.3);
  EXPECT_NEAR(out[2], -0.15, 1e-12);
}

TEST(HeadingHold, CorrectionIsClamped)
{
  tb::HeadingHold hold(1.5, 0.3);
  const auto out = hold.update({-0.2, 0.0, 0.0}, kUpright, {0.0, 0.0, -10.0}, 0.1);  // -1 rad
  EXPECT_DOUBLE_EQ(out[2], 0.3);
}

TEST(HeadingHold, TurningAndStandingPassThroughAndMoveTheReference)
{
  tb::HeadingHold hold(1.5, 0.3);
  // Operator turns while walking: command passes through, reference follows the heading.
  auto out = hold.update({0.3, 0.0, 0.4}, kUpright, {0.0, 0.0, 4.0}, 0.1);
  EXPECT_DOUBLE_EQ(out[2], 0.4);
  EXPECT_NEAR(hold.reference(), 0.4, 1e-12);
  // Standing: no correction, even though the robot rotated.
  out = hold.update({0.0, 0.0, 0.0}, kUpright, {0.0, 0.0, 2.0}, 0.1);
  EXPECT_DOUBLE_EQ(out[2], 0.0);
  EXPECT_NEAR(hold.reference(), 0.6, 1e-12);
  // Walking again holds the heading it had when it started walking.
  out = hold.update({0.3, 0.0, 0.0}, kUpright, kStill, 0.02);
  EXPECT_NEAR(out[2], 0.0, 1e-12);
}

TEST(HeadingHold, UsesYawAboutWorldVerticalWhenTilted)
{
  tb::HeadingHold hold(1.0, 10.0);
  // Base pitched 90 deg nose down: world up is base -x, so a base -x rotation is a left turn.
  const auto out = hold.update({0.3, 0.0, 0.0}, {1.0, 0.0, 0.0}, {-2.0, 0.0, 0.0}, 0.1);
  EXPECT_NEAR(hold.heading(), 0.2, 1e-12);
  EXPECT_NEAR(out[2], -0.2, 1e-12);  // turns back toward the start heading (0)
}

TEST(HeadingHold, WrapsAroundPi)
{
  tb::HeadingHold hold(1.0, 10.0);
  hold.update({0.0, 0.0, 0.0}, kUpright, {0.0, 0.0, 31.0}, 0.1);  // stand, turned to 3.1 rad
  const auto out = hold.update({0.3, 0.0, 0.0}, kUpright, {0.0, 0.0, 1.0}, 0.1);  // 3.2 -> -3.08
  EXPECT_NEAR(out[2], -0.1, 1e-9);  // short way back, not 2*pi the other way
}

TEST(HeadingHold, StallDoesNotStretchOneGyroSample)
{
  tb::HeadingHold hold(1.5, 0.3);
  hold.update({0.3, 0.0, 0.0}, kUpright, {0.0, 0.0, 0.5}, 0.7);  // 0.7 s late tick
  EXPECT_NEAR(hold.heading(), 0.05, 1e-12);  // integrated over 0.1 s, not 0.7 s
}

TEST(HeadingHold, RejectsBadParameters)
{
  EXPECT_THROW(tb::HeadingHold(-1.0, 0.3), std::invalid_argument);
  EXPECT_THROW(tb::HeadingHold(1.0, std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}

TEST(FreezeWatch, FrozenOnlyWhenUnchangedWhileTheTargetMoves)
{
  tb::FreezeWatch w;
  w.update(1.0, 0.0);
  w.observe_ref(0.0);
  w.observe_ref(0.05);
  EXPECT_FALSE(w.frozen(1.0, 0.5, 0.1));  // target moved only 0.05: a joint holding still
  w.observe_ref(0.2);
  EXPECT_FALSE(w.frozen(0.4, 0.5, 0.1));  // not long enough yet
  EXPECT_TRUE(w.frozen(0.6, 0.5, 0.1));   // 0.6 s unchanged while the target moved 0.2
  w.update(1.0001, 0.7);                  // feedback changed: starts over
  EXPECT_FALSE(w.frozen(2.0, 0.5, 0.1));  // and no target since
  w.observe_ref(0.2);
  EXPECT_FALSE(w.frozen(2.0, 0.5, 0.1));  // a steady target is no move
}

TEST(FreezeWatch, TargetsFromBeforeAResetDoNotCount)
{
  tb::FreezeWatch w;
  w.update(2.0, 0.0);
  w.observe_ref(0.0);                     // e.g. the last STAND target
  w.reset();                              // WALK starts
  w.update(2.0, 1.0);
  w.observe_ref(0.6);                     // first WALK targets, steady
  w.observe_ref(0.6);
  EXPECT_FALSE(w.frozen(3.0, 0.5, 0.1));
}

TEST(FreezeWatch, RangeCountsBothWaysAndTimeoutZeroDisables)
{
  tb::FreezeWatch w;
  w.update(2.0, 0.0);
  w.observe_ref(0.06);
  w.observe_ref(-0.06);                   // swung back and forth, 0.12 in total
  EXPECT_TRUE(w.frozen(1.0, 0.5, 0.1));
  EXPECT_FALSE(w.frozen(1.0, 0.0, 0.1));  // freeze_timeout 0 = off
  w.reset();
  EXPECT_FALSE(w.frozen(9.0, 0.5, 0.1));
}

TEST(VectorFreezeWatch, IdenticalVectorsFreezeAnyChangeResets)
{
  tb::VectorFreezeWatch w;
  w.update({0.1, 0.2}, 0.0);
  w.update({0.1, 0.2}, 0.3);
  EXPECT_FALSE(w.frozen(0.4, 0.5));
  EXPECT_TRUE(w.frozen(0.6, 0.5));
  w.update({0.1, 0.2000001}, 0.6);
  EXPECT_FALSE(w.frozen(1.0, 0.5));
  EXPECT_FALSE(w.frozen(1.0, 0.0));
}

TEST(OverLimitWatch, TripsOnlyAfterTheHoldAndResetsBelowTheLimit)
{
  tb::OverLimitWatch w;
  w.update(12.0, 10.0, 0.00);
  EXPECT_FALSE(w.tripped(0.03, 0.05));    // over, not long enough
  w.update(-13.0, 10.0, 0.03);            // sign does not matter
  EXPECT_TRUE(w.tripped(0.06, 0.05));
  EXPECT_DOUBLE_EQ(w.peak, 13.0);
  w.update(9.0, 10.0, 0.07);              // back under: starts over
  EXPECT_FALSE(w.tripped(1.0, 0.05));
  w.update(11.0, 10.0, 1.0);              // a single spike
  w.update(2.0, 10.0, 1.01);
  EXPECT_FALSE(w.tripped(2.0, 0.05));
  w.update(50.0, 0.0, 3.0);               // limit 0 = off
  EXPECT_FALSE(w.tripped(9.0, 0.05));
}

TEST(HeldGaitClock, StillWhenStandingAdvancesWhenWalkingRestartsFromZero)
{
  tb::HeldGaitClock c;
  const std::array<double, 3> stand{0.0, 0.0, 0.0}, drift{0.04, 0.03, 0.0}, walk{0.3, 0.0, 0.0};
  EXPECT_DOUBLE_EQ(c.step(walk, 0.02, 0.72, 0.1), 0.0);       // first WALK step: phase 0
  EXPECT_NEAR(c.step(walk, 0.02, 0.72, 0.1), 0.02 / 0.72, 1e-12);
  EXPECT_NEAR(c.step(walk, 0.02, 0.72, 0.1), 0.04 / 0.72, 1e-12);
  EXPECT_DOUBLE_EQ(c.step(stand, 0.02, 0.72, 0.1), 0.0);      // stand command: held at 0
  EXPECT_DOUBLE_EQ(c.step(drift, 0.02, 0.72, 0.1), 0.0);      // stick drift (|v| = 0.05) counts as standing
  EXPECT_NEAR(c.step(walk, 0.02, 0.72, 0.1), 0.02 / 0.72, 1e-12);  // walking again: from 0
  for (int i = 0; i < 36; ++i) {c.step(walk, 0.02, 0.72, 0.1);}
  EXPECT_NEAR(c.phase, 0.02 / 0.72, 1e-9);                    // wraps after one period (37 steps)
  EXPECT_NEAR(c.step({0.0, 0.0, 0.2}, 0.02, 0.72, 0.1), 0.04 / 0.72, 1e-9);  // turning in place walks
  c.reset();
  EXPECT_DOUBLE_EQ(c.step(walk, 0.02, 0.72, 0.1), 0.0);
}

TEST(CenteringTrim, LeansTowardTheTorqueSlowlyClampsAndFreezesDuringPushes)
{
  tb::CenteringTrim t;
  EXPECT_DOUBLE_EQ(t.update(2.0, 2.0, 0.02), 0.0);  // gain 0 = off
  t.gain = 0.0015;
  t.reset();
  for (int i = 0; i < 50; ++i) {t.update(1.75, 1.75, 0.02);}  // COM forward: steady +1.75 N m
  EXPECT_GT(t.lean, 0.0);                       // lean back (ankles toes down)
  EXPECT_LT(t.lean, 0.002);                     // but slowly: < 0.002 rad after 1 s
  const double before = t.lean;
  t.update(5.7, 5.7, 0.02);                     // a push: ankles at the foot-edge limit
  EXPECT_DOUBLE_EQ(t.lean, before);             // frozen, no windup
  for (int i = 0; i < 100000; ++i) {t.update(5.0, 5.0, 0.02);}
  EXPECT_DOUBLE_EQ(t.lean, t.max_lean);         // clamped
  t.reset();
  for (int i = 0; i < 500; ++i) {t.update(-1.0, -1.0, 0.02);}
  EXPECT_LT(t.lean, 0.0);                       // COM back: lean forward
}

TEST(ManualLean, StepsClampsAndFollowsAtTheRate)
{
  tb::ManualLean l;
  l.nudge(-1);
  l.nudge(-1);
  EXPECT_DOUBLE_EQ(l.target, -0.02);
  EXPECT_DOUBLE_EQ(l.applied, 0.0);
  EXPECT_NEAR(l.update(0.02), -0.001, 1e-12);   // 0.05 rad/s x 20 ms: no jump
  for (int i = 0; i < 100; ++i) {l.update(0.02);}
  EXPECT_DOUBLE_EQ(l.applied, -0.02);
  for (int i = 0; i < 100; ++i) {l.nudge(+1);}
  EXPECT_DOUBLE_EQ(l.target, l.max);            // clamped
  l.target = -0.08;
  l.snap();
  EXPECT_DOUBLE_EQ(l.applied, -0.08);           // STAND entry ramps straight into the lean
}

TEST(CenteringTrim, HoldsATargetTorqueInsteadOfZero)
{
  tb::CenteringTrim t;
  t.gain = 0.0015;
  t.max_lean = 0.03;
  t.target_torque = 1.5;
  t.reset();
  EXPECT_DOUBLE_EQ(t.filtered, 1.5);            // starts on target: no transient
  for (int i = 0; i < 500; ++i) {t.update(1.5, 1.5, 0.02);}
  EXPECT_DOUBLE_EQ(t.lean, 0.0);                // at the target torque: no lean
  for (int i = 0; i < 500; ++i) {t.update(0.5, 0.5, 0.02);}
  EXPECT_LT(t.lean, 0.0);                       // below target (weight too far back): lean forward
  t.reset();
  for (int i = 0; i < 500; ++i) {t.update(2.5, 2.5, 0.02);}
  EXPECT_GT(t.lean, 0.0);                       // above target: lean back
  for (int i = 0; i < 200000; ++i) {t.update(4.0, 4.0, 0.02);}
  EXPECT_DOUBLE_EQ(t.lean, 0.03);               // clamped to its own small max
}

TEST(TrimGate, WaitsForItsButtonAndRearmsOnEveryStand)
{
  tb::TrimGate g;
  EXPECT_TRUE(g.active());                      // no gate configured: old behavior, always on
  g.enabled = true;
  EXPECT_FALSE(g.active());                     // gated: waits
  g.arm();
  EXPECT_TRUE(g.active());                      // button pressed
  g.reset();                                    // new STAND or disarm
  EXPECT_FALSE(g.active());
  g.arm();
  g.arm();
  EXPECT_TRUE(g.active());
}

TEST(SquatMotion, ZeroAtTheEndsMonotonicDownAndUpAndSlow)
{
  tb::SquatMotion s;
  s.depth = 0.02;
  s.down_time = 3.0;
  s.hold_time = 1.0;
  // table endpoints and interpolation
  const auto z0 = tb::SquatMotion::at_depth(0.0);
  EXPECT_DOUBLE_EQ(z0[0], 0.0);
  EXPECT_DOUBLE_EQ(z0[1], 0.0);
  EXPECT_DOUBLE_EQ(z0[2], 0.0);
  const auto mid = tb::SquatMotion::at_depth(0.0025);   // halfway between the 0 and 0.5 cm rows
  EXPECT_NEAR(mid[1], 0.5 * 0.081702, 1e-12);
  EXPECT_DOUBLE_EQ(tb::SquatMotion::at_depth(1.0)[1], tb::SquatMotion::kTable.back().knee);  // clamped
  // timing
  EXPECT_DOUBLE_EQ(s.depth_at(-1.0), 0.0);
  EXPECT_DOUBLE_EQ(s.depth_at(3.5), 0.02);
  EXPECT_DOUBLE_EQ(s.depth_at(7.0), 0.0);
  EXPECT_FALSE(s.running);
  EXPECT_DOUBLE_EQ(s.offsets(5.0)[1], 0.0);              // not started: zero
  s.begin(10.0);
  double prev_knee = 0.0, max_rate = 0.0;
  for (int i = 1; i <= 150; ++i) {                        // down phase, 50 Hz
    const auto o = s.offsets(10.0 + i * 0.02);
    EXPECT_GE(o[1], prev_knee - 1e-12);                   // knee bends monotonically
    EXPECT_LE(o[0], 1e-12);                               // hip offset <= 0
    EXPECT_LE(o[2], 1e-12);                               // ankle offset <= 0
    max_rate = std::max(max_rate, std::abs(o[1] - prev_knee) / 0.02);
    prev_knee = o[1];
  }
  EXPECT_NEAR(prev_knee, 0.268828, 1e-6);                 // bottom = the 2 cm row
  EXPECT_LT(max_rate, 0.3);                               // rad/s, far below the 2 rad/s trip
  for (int i = 151; i <= 349; ++i) {
    const auto o = s.offsets(10.0 + i * 0.02);
    max_rate = std::max(max_rate, std::abs(o[1] - prev_knee) / 0.02);
    if (i > 200) {EXPECT_LE(o[1], prev_knee + 1e-12);}  // up phase: knee straightens monotonically
    prev_knee = o[1];
  }
  EXPECT_LT(max_rate, 0.3);
  const auto end = s.offsets(17.0);                       // past the 7 s cycle
  EXPECT_DOUBLE_EQ(end[0], 0.0);
  EXPECT_DOUBLE_EQ(end[1], 0.0);
  EXPECT_DOUBLE_EQ(end[2], 0.0);
  EXPECT_FALSE(s.running);
  s.begin(20.0);
  s.stop();                                               // B / disarm / new STAND
  EXPECT_DOUBLE_EQ(s.offsets(21.0)[1], 0.0);
}
