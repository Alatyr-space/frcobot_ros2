#ifndef FAIRINO_HARDWARE__RESUME_HOLD_HPP_
#define FAIRINO_HARDWARE__RESUME_HOLD_HPP_

// Q-C1 (a), F-226, the owner's answer of 2026-10-10: once the controller has
// refused ServoJ (code 14: the arm jogged at the pendant, or faulted), the
// trajectory controller's command is still where the arm was before, and the
// moment the controller accepts again it would execute it -- a jump back to
// the pre-jog pose. So, from the episode's start until the command is back
// within `max_step_rad` of the measured position on every joint, write()
// sends a hold instead of the command. The hold only ever withholds motion;
// the next goal starts from the measured state, so it lets go by itself.
//
// AN-023 (bench visit 30, 2026-10-26): the hold used to send the *measured*
// position, read again every cycle. That closes a loop through the
// controller: each ServoJ target is the position measured one or two cycles
// before (the state port showed the target equal to the previous frame's
// measurement on 451 of 2881 frames, joint 3's velocity swinging +-1 deg/s
// at a steady amplitude for 28 s), and the arm limit-cycles until the Arm
// module restarts. The hold's target is now LATCHED: it follows the
// measurement only while the controller is refusing (the arm is being moved
// by the pendant), and freezes at the first cycle the controller accepts
// again. A still arm is sent one fixed target, never its own measurement.
//
// Pure: no ROS, no SDK. Six joints, radians.

#include <algorithm>
#include <cmath>

namespace fairino_hardware
{

class ResumeHold
{
public:
  static constexpr int JOINTS = 6;

  explicit ResumeHold(double max_step_rad = 0.01)
  : max_step_rad_(max_step_rad) {}

  // A refusal episode began: hold until the command is near the arm.
  void episodeStarted() {armed_ = true;}

  // Back to the state on_activate wants: nothing held, nothing latched.
  void reset()
  {
    armed_ = false;
    latched_ = false;
    farthest_ = 0.0;
  }

  // One write cycle. `command` is the controller's, `measured` the arm's;
  // `refusing` whether the last ServoJ was refused (the episode still
  // active, as of the previous cycle's answer). Fills `target` with what to
  // send, and returns whether it is the hold's rather than the command.
  bool apply(
    const double command[JOINTS], const double measured[JOINTS], bool refusing,
    double target[JOINTS])
  {
    if (!armed_) {
      std::copy(command, command + JOINTS, target);
      return false;
    }
    farthest_ = 0.0;
    for (int j = 0; j < JOINTS; ++j) {
      farthest_ = std::max(farthest_, std::abs(command[j] - measured[j]));
    }
    if (farthest_ <= max_step_rad_) {
      // The command is near the arm: it goes as it is, and once the
      // controller accepts again the episode's hold is over.
      if (!refusing) {
        armed_ = false;
        latched_ = false;
      }
      std::copy(command, command + JOINTS, target);
      return false;
    }
    // Far: hold. The latch follows the measurement while the controller
    // refuses (the arm is being moved), and freezes once it accepts, so the
    // measurement is never fed back to a still arm.
    if (refusing || !latched_) {
      std::copy(measured, measured + JOINTS, latch_);
      latched_ = true;
    }
    std::copy(latch_, latch_ + JOINTS, target);
    return true;
  }

  bool armed() const {return armed_;}
  bool latched() const {return latched_;}
  double maxStepRad() const {return max_step_rad_;}
  // How far the command was from the arm at the last apply(), for the log.
  double farthest() const {return farthest_;}

private:
  double max_step_rad_;
  bool armed_ = false;
  bool latched_ = false;
  double farthest_ = 0.0;
  double latch_[JOINTS]{0, 0, 0, 0, 0, 0};
};

}  // namespace fairino_hardware

#endif  // FAIRINO_HARDWARE__RESUME_HOLD_HPP_
