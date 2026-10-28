// AN-023 (bench visit 30, 2026-10-26): after a pendant jog, with the Arm
// module running, the arm oscillated until the module was restarted. The
// state port showed the ServoJ target equal to the position measured one
// frame before: write()'s resume hold (Q-C1 (a), fork 10e7429) sent the
// measured position, read again every cycle, and so closed a loop through
// the controller and the arm's own motion. These rows hold the policy alone,
// no ROS: a still arm is sent one target, the hold's purpose is kept (no
// jump back to the pre-jog pose, let go once the command is near), and the
// latch follows the arm while the controller refuses.

#include <gmock/gmock.h>

#include <cmath>
#include <vector>

#include "fairino_hardware/resume_hold.hpp"

namespace
{

using fairino_hardware::ResumeHold;

constexpr double DEG = M_PI / 180.0;

struct Arm
{
  double command[6]{0.1, -0.2, 0.3, -0.4, 0.5, -0.6};
  double measured[6]{0.1, -0.2, 0.3, -0.4, 0.5, -0.6};
  double target[6]{0, 0, 0, 0, 0, 0};
};

// A pendant jog of `jog` radians on joint 3 over five refused cycles, then
// the controller accepts once (the hold not yet told): what that first
// accepted ServoJ carries is where the controller puts the arm.
void jogged(ResumeHold & hold, Arm & arm, double jog)
{
  hold.episodeStarted();
  for (int cycle = 0; cycle < 5; ++cycle) {
    arm.measured[2] += jog / 5;
    hold.apply(arm.command, arm.measured, true, arm.target);
  }
  hold.apply(arm.command, arm.measured, true, arm.target);
}

// The plant: the controller executes the last target with one cycle of
// delay, and the measurement carries +-noise. The targets the hold sends
// over `cycles` still cycles.
std::vector<double> stillCycles(ResumeHold & hold, Arm & arm, int cycles, double noise)
{
  std::vector<double> sent;
  for (int cycle = 0; cycle < cycles; ++cycle) {
    for (int j = 0; j < 6; ++j) {
      arm.measured[j] = arm.target[j] + ((cycle % 2) ? noise : -noise);
    }
    hold.apply(arm.command, arm.measured, false, arm.target);
    sent.push_back(arm.target[2]);
  }
  return sent;
}

TEST(ResumeHold, UntouchedItSendsTheCommand)
{
  ResumeHold hold(0.01);
  Arm arm;
  arm.measured[2] += 0.5;  // far, but no episode
  EXPECT_FALSE(hold.apply(arm.command, arm.measured, false, arm.target));
  EXPECT_DOUBLE_EQ(arm.target[2], arm.command[2]);
}

TEST(ResumeHold, AStillArmAfterAnEpisodeIsSentOneTarget)
{
  ResumeHold hold(0.01);
  Arm arm;
  jogged(hold, arm, 1.0 * DEG);
  const double frozen = arm.target[2];
  const auto sent = stillCycles(hold, arm, 300, 0.01 * DEG);
  for (size_t i = 0; i < sent.size(); ++i) {
    EXPECT_DOUBLE_EQ(sent[i], frozen) << "cycle " << i << ": the measurement fed back";
  }
  EXPECT_TRUE(hold.armed()) << "the command is still a degree away: the hold stands";
  EXPECT_TRUE(hold.latched());
}

TEST(ResumeHold, TheHoldIsWhereTheArmWasJoggedTo)
{
  ResumeHold hold(0.01);
  Arm arm;
  const double before = arm.measured[2];
  jogged(hold, arm, 1.0 * DEG);
  EXPECT_NEAR(arm.target[2], before + 1.0 * DEG, 1e-12) << "never the pre-jog command";
  EXPECT_NEAR(hold.farthest(), 1.0 * DEG, 1e-12);
  EXPECT_TRUE(hold.apply(arm.command, arm.measured, false, arm.target)) << "still held";
}

TEST(ResumeHold, LetsGoOnceTheCommandIsNearEveryJoint)
{
  ResumeHold hold(0.01);
  Arm arm;
  jogged(hold, arm, 1.0 * DEG);
  stillCycles(hold, arm, 10, 0.0);
  // The next goal starts from the measured state.
  for (int j = 0; j < 6; ++j) {
    arm.command[j] = arm.measured[j] + 0.005;
  }
  EXPECT_FALSE(hold.apply(arm.command, arm.measured, false, arm.target));
  EXPECT_DOUBLE_EQ(arm.target[2], arm.command[2]);
  EXPECT_FALSE(hold.armed());
  EXPECT_FALSE(hold.latched());
  // And a far command with no new episode goes as it is.
  arm.command[2] += 0.2;
  EXPECT_FALSE(hold.apply(arm.command, arm.measured, false, arm.target));
  EXPECT_DOUBLE_EQ(arm.target[2], arm.command[2]);
}

TEST(ResumeHold, ANearCommandWhileStillRefusedGoesAsItIsAndTheHoldStands)
{
  ResumeHold hold(0.01);
  Arm arm;
  hold.episodeStarted();
  arm.measured[2] += 0.005;
  EXPECT_FALSE(hold.apply(arm.command, arm.measured, true, arm.target));
  EXPECT_DOUBLE_EQ(arm.target[2], arm.command[2]);
  EXPECT_TRUE(hold.armed()) << "not over until the controller accepts";
}

TEST(ResumeHold, TheLatchFollowsTheArmWhileTheControllerRefuses)
{
  ResumeHold hold(0.01);
  Arm arm;
  jogged(hold, arm, 1.0 * DEG);
  stillCycles(hold, arm, 10, 0.0);
  const double first = arm.target[2];
  // A second jog: refused again, the arm moving.
  hold.episodeStarted();
  arm.measured[2] += 0.5 * DEG;
  EXPECT_TRUE(hold.apply(arm.command, arm.measured, true, arm.target));
  EXPECT_DOUBLE_EQ(arm.target[2], arm.measured[2]) << "following the jog";
  arm.measured[2] += 0.5 * DEG;
  EXPECT_TRUE(hold.apply(arm.command, arm.measured, true, arm.target));
  EXPECT_DOUBLE_EQ(arm.target[2], arm.measured[2]);
  // Accepted: frozen at the last refused cycle's measurement, whatever the
  // measurement does next.
  const double second = arm.target[2];
  EXPECT_NE(second, first);
  arm.measured[2] += 0.01 * DEG;
  EXPECT_TRUE(hold.apply(arm.command, arm.measured, false, arm.target));
  EXPECT_DOUBLE_EQ(arm.target[2], second);
}

TEST(ResumeHold, ResetForgetsEverything)
{
  ResumeHold hold(0.01);
  Arm arm;
  jogged(hold, arm, 1.0 * DEG);
  hold.reset();
  EXPECT_FALSE(hold.armed());
  EXPECT_FALSE(hold.latched());
  EXPECT_FALSE(hold.apply(arm.command, arm.measured, false, arm.target));
  EXPECT_DOUBLE_EQ(arm.target[2], arm.command[2]);
}

TEST(ResumeHold, TheStepIsTheOneGiven)
{
  ResumeHold wide(0.3);
  Arm arm;
  wide.episodeStarted();
  arm.measured[2] += 0.2;
  EXPECT_FALSE(wide.apply(arm.command, arm.measured, false, arm.target));
  EXPECT_DOUBLE_EQ(wide.maxStepRad(), 0.3);
}

}  // namespace
