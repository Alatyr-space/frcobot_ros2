#ifndef _FR_HARDWARE_INTERFACE_
#define _FR_HARDWARE_INTERFACE_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "visibility_control.h"
#include <vector>
#include "libfairino/include/robot.h"
#include "fairino_hardware/robot_sdk.hpp"


#define CONTROLLER_IP_ADDRESS "192.168.58.2"

namespace fairino_hardware
{

class FairinoHardwareInterface: public hardware_interface::SystemInterface{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FairinoHardwareInterface)

  // pluginlib's: the real SDK, through makeRobotSdk().
  FAIRINO_HARDWARE_PUBLIC
  FairinoHardwareInterface();

  // A test's: the SDK the factory yields, built in on_activate as the real
  // one would be.
  FAIRINO_HARDWARE_PUBLIC
  explicit FairinoHardwareInterface(RobotSdkFactory sdk_factory);

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  //FAIRINO_HARDWARE_PUBLIC
  //hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  
  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  
  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  // hardware_interface::return_type prepare_command_mode_switch(
  //   const std::vector<std::string> & start_interfaces,
  //   const std::vector<std::string> & stop_interfaces) override;
  // hardware_interface::return_type perform_command_mode_switch(
  //   const std::vector<std::string>& start_interfaces,
  //   const std::vector<std::string>& stop_interfaces) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
private:
  double _jnt_position_command[6];
  double _jnt_velocity_command[6];
  double _jnt_torque_command[6];
  double _jnt_position_state[6];
  double _jnt_velocity_state[6];
  double _jnt_torque_state[6];
  // Position control until on_activate says otherwise; uninitialised, write()
  // read an indeterminate mode before activation.
  int _control_mode = 0;
  // F-204: whether the description declares a velocity state interface.
  // Exported, and read from the robot's feedback, only then.
  bool _export_velocity = false;
  std::string _controller_ip = CONTROLLER_IP_ADDRESS;
  RobotSdkFactory _sdk_factory;
  std::unique_ptr<RobotSdk> _ptr_robot;

  // F-323: a -2 (ERR_SOCKET_COM_FAILED) on a read is held through, the last
  // good state kept, for up to this long (read()'s own `time`) from the first
  // failed cycle; past it, read() fails as it always did. One good cycle ends
  // the episode. Any other code fails at once.
  int64_t _transient_read_error_tolerance_ns = 100 * 1000000;
  bool _read_failing = false;
  rclcpp::Time _read_failing_since;
  int _read_failures = 0;
  // Whether this cycle's read may go on; false past the bound or on a code
  // that is not transient. `what` names the read, for the log.
  bool acceptReadCode(errno_t code, const char * what, const rclcpp::Time & time);
};

} //end namespace


#endif