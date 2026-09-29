// F-204 and F-184: the driver exports the robot-measured joint velocity when
// the description declares it, and write() refuses a non-finite command on
// every joint. None of these cases reaches the robot: on_init and the exports
// need no connection, and write() refuses before it would call ServoJ.

#include <gmock/gmock.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "fairino_hardware/fairino_hardware_interface.hpp"

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
