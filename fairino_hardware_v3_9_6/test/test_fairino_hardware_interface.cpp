// F-204 and F-184: the driver exports the robot-measured joint velocity when
// the description declares it, and write() refuses a non-finite command on
// every joint. None of these cases reaches the robot: on_init and the exports
// need no connection, and write() refuses before it would call ServoJ.
//
// From C1 on, a test that activates the driver gives it a FakeRobotSdk, and
// CMake runs this binary with FAIRINO_HARDWARE_TEST_NO_ROBOT=1, under which
// the real SDK refuses to be built: no test here can reach the controller.

#include <gmock/gmock.h>

#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "fairino_hardware/fairino_hardware_interface.hpp"
#include "fairino_hardware/robot_sdk.hpp"
#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace
{

using hardware_interface::CallbackReturn;
using hardware_interface::ComponentInfo;
using hardware_interface::HardwareInfo;
using hardware_interface::InterfaceInfo;

InterfaceInfo interfaceNamed(const std::string & name)
{
  InterfaceInfo info;
  info.name = name;
  return info;
}

// Six joints, a position command each, and the given state interfaces.
HardwareInfo robotWith(const std::vector<std::vector<std::string>> & statesPerJoint)
{
  HardwareInfo info;
  info.name = "FakeSystem";
  info.type = "system";
  info.hardware_plugin_name = "fairino_hardware/FairinoHardwareInterface";
  for (size_t i = 0; i < statesPerJoint.size(); ++i) {
    ComponentInfo joint;
    joint.name = "j" + std::to_string(i + 1);
    joint.type = "joint";
    joint.command_interfaces.push_back(interfaceNamed(hardware_interface::HW_IF_POSITION));
    for (const auto & state : statesPerJoint[i]) {
      joint.state_interfaces.push_back(interfaceNamed(state));
    }
    info.joints.push_back(joint);
  }
  return info;
}

const std::vector<std::string> POSITION_AND_VELOCITY{
  hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY};
const std::vector<std::string> POSITION_ONLY{hardware_interface::HW_IF_POSITION};

// What the fake SDK was asked, and what it answers. Shared with the test, so
// it outlives the fake (on_deactivate releases the SDK without deleting it).
struct SdkScript
{
  std::vector<std::string> calls;
  double joint_degrees[6]{10.0, -20.0, 30.0, -40.0, 50.0, -60.0};
  float speed_degrees[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // Codes to return, one per call, before falling back to 0.
  std::deque<int> position_codes;
  std::deque<int> speed_codes;
  std::deque<int> servo_codes;
  std::vector<std::vector<double>> servo_degrees;
};

class FakeRobotSdk : public fairino_hardware::RobotSdk
{
public:
  explicit FakeRobotSdk(std::shared_ptr<SdkScript> script)
  : script_(std::move(script)) {}

  errno_t RPC(const char * ip) override
  {
    script_->calls.push_back(std::string("RPC ") + ip);
    return 0;
  }
  errno_t CloseRPC() override {return record("CloseRPC");}
  errno_t StopMotion() override {return record("StopMotion");}
  errno_t GetActualJointPosDegree(uint8_t flag, JointPos * jPos) override
  {
    script_->calls.push_back("GetActualJointPosDegree " + std::to_string(flag));
    const int code = next(script_->position_codes);
    if (code == 0) {
      for (int j = 0; j < 6; ++j) {
        jPos->jPos[j] = script_->joint_degrees[j];
      }
    }
    return code;
  }
  errno_t GetActualJointSpeedsDegree(uint8_t flag, float speed[6]) override
  {
    script_->calls.push_back("GetActualJointSpeedsDegree " + std::to_string(flag));
    const int code = next(script_->speed_codes);
    if (code == 0) {
      for (int j = 0; j < 6; ++j) {
        speed[j] = script_->speed_degrees[j];
      }
    }
    return code;
  }
  errno_t ServoJ(
    JointPos * joint_pos, ExaxisPos *, float, float, float, float, float) override
  {
    script_->calls.push_back("ServoJ");
    script_->servo_degrees.emplace_back(joint_pos->jPos, joint_pos->jPos + 6);
    return next(script_->servo_codes);
  }

private:
  errno_t record(const std::string & call)
  {
    script_->calls.push_back(call);
    return 0;
  }
  static int next(std::deque<int> & codes)
  {
    if (codes.empty()) {
      return 0;
    }
    const int code = codes.front();
    codes.pop_front();
    return code;
  }

  std::shared_ptr<SdkScript> script_;
};

fairino_hardware::RobotSdkFactory fakeSdk(std::shared_ptr<SdkScript> script)
{
  return [script]() {return std::make_unique<FakeRobotSdk>(script);};
}

const rclcpp_lifecycle::State INACTIVE(
  lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");

std::vector<std::string> stateNames(fairino_hardware::FairinoHardwareInterface & driver)
{
  std::vector<std::string> names;
  for (const auto & state : driver.export_state_interfaces()) {
    names.push_back(state.get_name());
  }
  return names;
}

}  // namespace

TEST(FairinoHardwareInterface, ExportsVelocityWhenEveryJointDeclaresIt)
{
  fairino_hardware::FairinoHardwareInterface driver;
  ASSERT_EQ(
    driver.on_init(robotWith(std::vector<std::vector<std::string>>(6, POSITION_AND_VELOCITY))),
    CallbackReturn::SUCCESS);

  const auto names = stateNames(driver);
  EXPECT_EQ(names.size(), 12u);
  for (int j = 1; j <= 6; ++j) {
    EXPECT_THAT(names, testing::Contains("j" + std::to_string(j) + "/position"));
    EXPECT_THAT(names, testing::Contains("j" + std::to_string(j) + "/velocity"));
  }
}

TEST(FairinoHardwareInterface, APositionOnlyDescriptionStillLoads)
{
  fairino_hardware::FairinoHardwareInterface driver;
  ASSERT_EQ(
    driver.on_init(robotWith(std::vector<std::vector<std::string>>(6, POSITION_ONLY))),
    CallbackReturn::SUCCESS);

  const auto names = stateNames(driver);
  EXPECT_EQ(names.size(), 6u);
  EXPECT_THAT(names, testing::Not(testing::Contains("j1/velocity")));
}

TEST(FairinoHardwareInterface, RefusesVelocityOnSomeJointsOnly)
{
  std::vector<std::vector<std::string>> states(6, POSITION_AND_VELOCITY);
  states[5] = POSITION_ONLY;
  fairino_hardware::FairinoHardwareInterface driver;
  EXPECT_EQ(driver.on_init(robotWith(states)), CallbackReturn::ERROR);
}

TEST(FairinoHardwareInterface, RefusesASecondStateInterfaceThatIsNotVelocity)
{
  fairino_hardware::FairinoHardwareInterface driver;
  EXPECT_EQ(
    driver.on_init(robotWith(std::vector<std::vector<std::string>>(
      6, {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_EFFORT}))),
    CallbackReturn::ERROR);
}

// The guard read j1 to j5 only (`&cmd[0], &cmd[5]`, an exclusive end), so a
// NaN on j6 alone went on to ServoJ. Unfixed, this test does not return
// ERROR: it reaches the robot handle, which is null outside on_activate.
TEST(FairinoHardwareInterface, WriteRefusesANonFiniteCommandOnEveryJoint)
{
  for (int bad = 0; bad < 6; ++bad) {
    fairino_hardware::FairinoHardwareInterface driver;
    ASSERT_EQ(
      driver.on_init(robotWith(std::vector<std::vector<std::string>>(6, POSITION_AND_VELOCITY))),
      CallbackReturn::SUCCESS);
    auto commands = driver.export_command_interfaces();
    ASSERT_EQ(commands.size(), 6u);
    for (int j = 0; j < 6; ++j) {
      (void)commands[j].set_value(j == bad ? std::numeric_limits<double>::quiet_NaN() : 0.0);
    }
    EXPECT_EQ(
      driver.write(rclcpp::Time(0, 0), rclcpp::Duration(0, 10000000)),
      hardware_interface::return_type::ERROR) << "a NaN on j" << bad + 1;
  }
}

// C1: the seam. The driver activates on the SDK its factory yields, and the
// default factory refuses to build the real one under the test guard.
TEST(FairinoHardwareInterface, ActivatesOnTheFakeSdkAndNeverBuildsTheRealOne)
{
  auto script = std::make_shared<SdkScript>();
  fairino_hardware::FairinoHardwareInterface driver(fakeSdk(script));
  ASSERT_EQ(
    driver.on_init(robotWith(std::vector<std::vector<std::string>>(6, POSITION_AND_VELOCITY))),
    CallbackReturn::SUCCESS);
  auto commands = driver.export_command_interfaces();

  ASSERT_EQ(driver.on_activate(INACTIVE), CallbackReturn::SUCCESS);
  EXPECT_THAT(
    script->calls,
    testing::ElementsAre("RPC 192.168.58.2", "GetActualJointPosDegree 0"));
  // The command starts where the arm is.
  EXPECT_NEAR(commands[0].get_optional().value(), 10.0 / 180.0 * M_PI, 1e-12);
  EXPECT_NEAR(commands[5].get_optional().value(), -60.0 / 180.0 * M_PI, 1e-12);

  EXPECT_THROW(fairino_hardware::makeRobotSdk(), std::runtime_error);
  fairino_hardware::FairinoHardwareInterface real;
  ASSERT_EQ(
    real.on_init(robotWith(std::vector<std::vector<std::string>>(6, POSITION_AND_VELOCITY))),
    CallbackReturn::SUCCESS);
  EXPECT_EQ(real.on_activate(INACTIVE), CallbackReturn::ERROR);
}

// C1: pluginlib needs the default constructor; the plugin still loads by
// name. It is never activated, so the guard is not even reached.
TEST(FairinoHardwareInterface, ThePluginStillLoadsByName)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  auto driver = loader.createUniqueInstance("fairino_hardware/FairinoHardwareInterface");
  ASSERT_NE(driver, nullptr);
}
