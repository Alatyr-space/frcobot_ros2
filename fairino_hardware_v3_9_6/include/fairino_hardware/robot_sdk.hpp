#ifndef FAIRINO_HARDWARE__ROBOT_SDK_HPP_
#define FAIRINO_HARDWARE__ROBOT_SDK_HPP_

// The calls the hardware interface makes on the vendor SDK, and nothing else.
//
// `FRRobot` is a concrete class built in on_activate, so no test could stand
// in for it. The driver now holds a `RobotSdk`: `FRRobotSdk` wraps the real
// one, and a test gives the driver a factory of its own. The interface lists
// only what the driver calls -- there is no ResetAllError here, so the driver
// cannot clear a controller fault (F-226: clearing a fault is the operator's
// act).

#include <cstdint>
#include <cstdlib>
#include <functional>
#include <memory>
#include <stdexcept>

#include "libfairino/include/robot.h"

namespace fairino_hardware
{

class RobotSdk
{
public:
  virtual ~RobotSdk() = default;
  virtual errno_t RPC(const char * ip) = 0;
  virtual errno_t CloseRPC() = 0;
  virtual errno_t StopMotion() = 0;
  virtual errno_t GetActualJointPosDegree(uint8_t flag, JointPos * jPos) = 0;
  virtual errno_t GetActualJointSpeedsDegree(uint8_t flag, float speed[6]) = 0;
  virtual errno_t ServoJ(
    JointPos * joint_pos, ExaxisPos * axisPos, float acc, float vel, float cmdT,
    float filterT, float gain) = 0;
};

// The real SDK behind the interface: each call forwarded, unchanged.
class FRRobotSdk : public RobotSdk
{
public:
  errno_t RPC(const char * ip) override {return robot_.RPC(ip);}
  errno_t CloseRPC() override {return robot_.CloseRPC();}
  errno_t StopMotion() override {return robot_.StopMotion();}
  errno_t GetActualJointPosDegree(uint8_t flag, JointPos * jPos) override
  {
    return robot_.GetActualJointPosDegree(flag, jPos);
  }
  errno_t GetActualJointSpeedsDegree(uint8_t flag, float speed[6]) override
  {
    return robot_.GetActualJointSpeedsDegree(flag, speed);
  }
  errno_t ServoJ(
    JointPos * joint_pos, ExaxisPos * axisPos, float acc, float vel, float cmdT,
    float filterT, float gain) override
  {
    return robot_.ServoJ(joint_pos, axisPos, acc, vel, cmdT, filterT, gain);
  }

private:
  FRRobot robot_;
};

using RobotSdkFactory = std::function<std::unique_ptr<RobotSdk>()>;

// What pluginlib's default constructor uses: the real SDK -- unless
// FAIRINO_HARDWARE_TEST_NO_ROBOT is set, which a test sets so that nothing it
// runs can reach the controller (192.168.58.2 is reachable from the bench PC).
inline std::unique_ptr<RobotSdk> makeRobotSdk()
{
  const char * guard = std::getenv("FAIRINO_HARDWARE_TEST_NO_ROBOT");
  if (guard != nullptr && guard[0] != '\0' && guard[0] != '0') {
    throw std::runtime_error(
      "FAIRINO_HARDWARE_TEST_NO_ROBOT is set: the real robot SDK is not built");
  }
  return std::make_unique<FRRobotSdk>();
}

}  // namespace fairino_hardware

#endif  // FAIRINO_HARDWARE__ROBOT_SDK_HPP_
