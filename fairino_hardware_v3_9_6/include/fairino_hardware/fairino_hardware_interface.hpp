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
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "fairino_hardware/refusal_episode.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include <functional>


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

  // Where the driver's diagnostic goes; true once it is on its way. A test's
  // sink sees each status; the default publishes on /diagnostics from the
  // component's node.
  using DiagnosticSink = std::function<bool (const diagnostic_msgs::msg::DiagnosticStatus &)>;

  FAIRINO_HARDWARE_PUBLIC
  FairinoHardwareInterface(RobotSdkFactory sdk_factory, DiagnosticSink diagnostic_sink);

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
  DiagnosticSink _diagnostic_sink;

  // F-226: ServoJ's refusals, as episodes. One ERROR line at the onset (and
  // at a change of code), one WARN at the end, and the diagnostic contract
  // on every change and once a second; write() goes on returning OK, as the
  // controller's refusal is not the driver's to act on.
  RefusalEpisode _refusal;
  std::shared_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>
    _diagnostic_publisher;
  void noteServoJ(int code, const rclcpp::Time & time);
  diagnostic_msgs::msg::DiagnosticStatus servoJStatus() const;

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