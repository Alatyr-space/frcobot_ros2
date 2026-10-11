// F-204 and F-184: the driver exports the robot-measured joint velocity when
// the description declares it, and write() refuses a non-finite command on
// every joint. None of these cases reaches the robot: on_init and the exports
// need no connection, and write() refuses before it would call ServoJ.
//
// From C1 on, a test that activates the driver gives it a FakeRobotSdk, and
// CMake runs this binary with FAIRINO_HARDWARE_TEST_NO_ROBOT=1, under which
// the real SDK refuses to be built: no test here can reach the controller.

#include <gmock/gmock.h>

#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "fairino_hardware/fairino_hardware_interface.hpp"
#include "fairino_hardware/robot_sdk.hpp"
#include "hardware_interface/system_interface.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "hardware_interface/types/hardware_component_params.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
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

// The driver activated on a fake SDK, and its exported interfaces, kept so a
// test can read what the controllers would.
struct ActiveDriver
{
  explicit ActiveDriver(bool velocity = true)
  : script(std::make_shared<SdkScript>()),
    driver(fakeSdk(script))
  {
    EXPECT_EQ(
      driver.on_init(robotWith(std::vector<std::vector<std::string>>(
        6, velocity ? POSITION_AND_VELOCITY : POSITION_ONLY))),
      CallbackReturn::SUCCESS);
    states = driver.export_state_interfaces();
    commands = driver.export_command_interfaces();
    EXPECT_EQ(driver.on_activate(INACTIVE), CallbackReturn::SUCCESS);
  }

  double state(const std::string & name)
  {
    for (auto & state : states) {
      if (state.get_name() == name) {
        return state.get_optional().value();
      }
    }
    throw std::out_of_range(name);
  }

  // read() at `ms` milliseconds of the controller manager's clock.
  hardware_interface::return_type readAt(int64_t ms)
  {
    return driver.read(rclcpp::Time(ms * 1000000), rclcpp::Duration(0, 10000000));
  }

  std::shared_ptr<SdkScript> script;
  fairino_hardware::FairinoHardwareInterface driver;
  std::vector<hardware_interface::StateInterface> states;
  std::vector<hardware_interface::CommandInterface> commands;
};

constexpr double DEG = M_PI / 180.0;

// F-323: one -2 (ERR_SOCKET_COM_FAILED) on the position read deactivated
// the arm at rest. It is held through, up to the bound, the last good state
// kept.
TEST(FairinoHardwareInterface, ReadHoldsThroughOneTransientSocketError)
{
  ActiveDriver arm;
  ASSERT_EQ(arm.readAt(0), hardware_interface::return_type::OK);
  EXPECT_NEAR(arm.state("j1/position"), 10.0 * DEG, 1e-12);

  arm.script->position_codes = {-2};
  arm.script->joint_degrees[0] = 11.0;
  EXPECT_EQ(arm.readAt(10), hardware_interface::return_type::OK);
  EXPECT_NEAR(arm.state("j1/position"), 10.0 * DEG, 1e-12) << "the last good value, held";

  EXPECT_EQ(arm.readAt(20), hardware_interface::return_type::OK);
  EXPECT_NEAR(arm.state("j1/position"), 11.0 * DEG, 1e-12) << "the next good read lands";
}

TEST(FairinoHardwareInterface, ReadFailsWhenTheErrorOutlastsItsBound)
{
  ActiveDriver arm;
  ASSERT_EQ(arm.readAt(0), hardware_interface::return_type::OK);
  arm.script->position_codes = std::deque<int>(20, -2);
  // 100 ms is the default bound: failures from 10 ms to 110 ms are held,
  // the one at 120 ms (110 ms after the first) is past it.
  for (int64_t ms = 10; ms <= 110; ms += 10) {
    EXPECT_EQ(arm.readAt(ms), hardware_interface::return_type::OK) << "at " << ms << " ms";
  }
  EXPECT_EQ(arm.readAt(120), hardware_interface::return_type::ERROR);
}

TEST(FairinoHardwareInterface, ReadFailsAtOnceOnAnotherCode)
{
  ActiveDriver arm;
  ASSERT_EQ(arm.readAt(0), hardware_interface::return_type::OK);
  arm.script->position_codes = {-4};
  EXPECT_EQ(arm.readAt(10), hardware_interface::return_type::ERROR);
}

TEST(FairinoHardwareInterface, ReadHoldsThroughOneTransientSocketErrorOnTheSpeeds)
{
  ActiveDriver arm;
  arm.script->speed_degrees[2] = 5.0;
  ASSERT_EQ(arm.readAt(0), hardware_interface::return_type::OK);
  EXPECT_NEAR(arm.state("j3/velocity"), 5.0 * DEG, 1e-6);

  arm.script->speed_codes = {-2};
  arm.script->speed_degrees[2] = 6.0;
  EXPECT_EQ(arm.readAt(10), hardware_interface::return_type::OK);
  EXPECT_NEAR(arm.state("j3/velocity"), 5.0 * DEG, 1e-6) << "the last good value, held";

  EXPECT_EQ(arm.readAt(20), hardware_interface::return_type::OK);
  EXPECT_NEAR(arm.state("j3/velocity"), 6.0 * DEG, 1e-6);
}

TEST(FairinoHardwareInterface, ReadFailsWhenTheSpeedErrorOutlastsItsBound)
{
  ActiveDriver arm;
  ASSERT_EQ(arm.readAt(0), hardware_interface::return_type::OK);
  arm.script->speed_codes = std::deque<int>(20, -2);
  for (int64_t ms = 10; ms <= 110; ms += 10) {
    EXPECT_EQ(arm.readAt(ms), hardware_interface::return_type::OK) << "at " << ms << " ms";
  }
  EXPECT_EQ(arm.readAt(120), hardware_interface::return_type::ERROR);
}

TEST(FairinoHardwareInterface, ReadFailsAtOnceOnAnotherSpeedCode)
{
  ActiveDriver arm;
  ASSERT_EQ(arm.readAt(0), hardware_interface::return_type::OK);
  arm.script->speed_codes = {-4};
  EXPECT_EQ(arm.readAt(10), hardware_interface::return_type::ERROR);
}

// A good cycle ends the episode: a second -2 later starts its own bound.
TEST(FairinoHardwareInterface, OneGoodReadResetsTheBound)
{
  ActiveDriver arm;
  ASSERT_EQ(arm.readAt(0), hardware_interface::return_type::OK);
  arm.script->position_codes = std::deque<int>(10, -2);  // 10 to 100 ms
  for (int64_t ms = 10; ms <= 100; ms += 10) {
    ASSERT_EQ(arm.readAt(ms), hardware_interface::return_type::OK);
  }
  ASSERT_EQ(arm.readAt(110), hardware_interface::return_type::OK);  // good
  arm.script->position_codes = std::deque<int>(10, -2);  // 120 to 210 ms
  for (int64_t ms = 120; ms <= 210; ms += 10) {
    EXPECT_EQ(arm.readAt(ms), hardware_interface::return_type::OK) << "at " << ms << " ms";
  }
}

// The bound is the description's <param>.
TEST(FairinoHardwareInterface, TheBoundIsAHardwareParameter)
{
  auto script = std::make_shared<SdkScript>();
  fairino_hardware::FairinoHardwareInterface driver(fakeSdk(script));
  auto info = robotWith(std::vector<std::vector<std::string>>(6, POSITION_ONLY));
  info.hardware_parameters["transient_read_error_tolerance_ms"] = "30";
  ASSERT_EQ(driver.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(driver.on_activate(INACTIVE), CallbackReturn::SUCCESS);
  ASSERT_EQ(driver.read(rclcpp::Time(0), rclcpp::Duration(0, 10000000)),
    hardware_interface::return_type::OK);
  script->position_codes = std::deque<int>(10, -2);
  for (int64_t ms = 10; ms <= 40; ms += 10) {
    EXPECT_EQ(driver.read(rclcpp::Time(ms * 1000000), rclcpp::Duration(0, 10000000)),
      hardware_interface::return_type::OK) << "at " << ms << " ms";
  }
  EXPECT_EQ(driver.read(rclcpp::Time(50 * 1000000), rclcpp::Duration(0, 10000000)),
    hardware_interface::return_type::ERROR);

  auto bad = robotWith(std::vector<std::vector<std::string>>(6, POSITION_ONLY));
  bad.hardware_parameters["transient_read_error_tolerance_ms"] = "-5";
  fairino_hardware::FairinoHardwareInterface refused(fakeSdk(script));
  EXPECT_EQ(refused.on_init(bad), CallbackReturn::ERROR);
}

// --- F-226: a refused ServoJ (C3) ---------------------------------------

// Every log line the driver writes while it lives, with its severity.
class LogCapture
{
public:
  LogCapture()
  : previous_(rcutils_logging_get_output_handler())
  {
    lines().clear();
    rcutils_logging_set_output_handler(&LogCapture::handle);
  }
  ~LogCapture() {rcutils_logging_set_output_handler(previous_);}

  // The lines of a severity that hold every one of the fragments.
  int count(int severity, const std::vector<std::string> & fragments) const
  {
    int n = 0;
    for (const auto & [level, text] : lines()) {
      bool all = level == severity;
      for (const auto & fragment : fragments) {
        all = all && text.find(fragment) != std::string::npos;
      }
      n += all ? 1 : 0;
    }
    return n;
  }
  int count(int severity) const {return count(severity, {});}

private:
  static std::vector<std::pair<int, std::string>> & lines()
  {
    static std::vector<std::pair<int, std::string>> captured;
    return captured;
  }
  static void handle(
    const rcutils_log_location_t *, int severity, const char *,
    rcutils_time_point_value_t, const char * format, va_list * args)
  {
    char text[1024];
    va_list copy;
    va_copy(copy, *args);
    std::vsnprintf(text, sizeof(text), format, copy);
    va_end(copy);
    lines().emplace_back(severity, text);
  }

  rcutils_logging_output_handler_t previous_;
};

// The driver activated on a fake SDK, with a diagnostic sink of the test's.
struct ServoingDriver
{
  ServoingDriver()
  : script(std::make_shared<SdkScript>()),
    driver(fakeSdk(script), [this](const diagnostic_msgs::msg::DiagnosticStatus & status) {
        published.push_back(status);
        return true;
      })
  {
    EXPECT_EQ(
      driver.on_init(robotWith(std::vector<std::vector<std::string>>(6, POSITION_AND_VELOCITY))),
      CallbackReturn::SUCCESS);
    states = driver.export_state_interfaces();
    commands = driver.export_command_interfaces();
    EXPECT_EQ(driver.on_activate(INACTIVE), CallbackReturn::SUCCESS);
  }

  // One control cycle at `ms`: read, then write.
  hardware_interface::return_type cycleAt(int64_t ms)
  {
    const rclcpp::Time time(ms * 1000000);
    EXPECT_EQ(driver.read(time, rclcpp::Duration(0, 10000000)), hardware_interface::return_type::OK);
    return driver.write(time, rclcpp::Duration(0, 10000000));
  }

  void command(int joint, double radians) {(void)commands[joint].set_value(radians);}

  std::shared_ptr<SdkScript> script;
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> published;
  fairino_hardware::FairinoHardwareInterface driver;
  std::vector<hardware_interface::StateInterface> states;
  std::vector<hardware_interface::CommandInterface> commands;
};

std::string valueOf(const diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & key)
{
  for (const auto & value : status.values) {
    if (value.key == key) {
      return value.value;
    }
  }
  return "<absent>";
}

// F-226: the controller refused ServoJ (code 14) every 10 ms, and the driver
// said so at INFO, 100 lines a second, while write() returned OK: the
// refusal was buried. One ERROR per episode (with the vendor's own text, so
// a grep for 错误码 still finds the onset), one WARN at its end.
TEST(FairinoHardwareInterface, ARefusalNeitherDeactivatesNorFloodsTheLog)
{
  ServoingDriver arm;
  LogCapture log;
  arm.script->servo_codes = std::deque<int>(100, 14);
  for (int64_t cycle = 0; cycle < 105; ++cycle) {
    EXPECT_EQ(arm.cycleAt(cycle * 10), hardware_interface::return_type::OK) << "cycle " << cycle;
  }
  EXPECT_EQ(
    log.count(RCUTILS_LOG_SEVERITY_ERROR, {"ServoJ refused, code 14", "错误码:14"}), 1);
  EXPECT_EQ(log.count(RCUTILS_LOG_SEVERITY_ERROR), 1);
  EXPECT_EQ(log.count(RCUTILS_LOG_SEVERITY_WARN, {"accepted again after 100 refusals"}), 1);
  EXPECT_EQ(log.count(RCUTILS_LOG_SEVERITY_INFO), 0) << "no line per cycle";
}

// A change of code within an episode is one more ERROR line, not a new episode.
TEST(FairinoHardwareInterface, AChangeOfCodeIsOneMoreErrorLine)
{
  ServoingDriver arm;
  LogCapture log;
  arm.script->servo_codes = {14, 14, 14, 7, 7, 7};
  for (int64_t cycle = 0; cycle < 8; ++cycle) {
    EXPECT_EQ(arm.cycleAt(cycle * 10), hardware_interface::return_type::OK);
  }
  EXPECT_EQ(log.count(RCUTILS_LOG_SEVERITY_ERROR, {"ServoJ refused, code 14", "错误码:14"}), 1);
  EXPECT_EQ(log.count(RCUTILS_LOG_SEVERITY_ERROR, {"ServoJ refused, code 7", "错误码:7"}), 1);
  EXPECT_EQ(log.count(RCUTILS_LOG_SEVERITY_WARN, {"accepted again after 6 refusals", "code 7"}), 1);
}

// The contract the motion capability reads (C5): an ERROR status at the
// onset, repeated no more than once a second of `time`, an OK status at
// acceptance, and the OK repeated once a second too, for a late subscriber.
TEST(FairinoHardwareInterface, ARefusalIsPublishedAsADiagnostic)
{
  ServoingDriver arm;
  arm.script->servo_codes = std::deque<int>(250, 14);  // 1010 to 3500 ms
  arm.script->servo_codes.push_front(0);  // 1000 ms
  for (int64_t cycle = 0; cycle < 260; ++cycle) {
    arm.cycleAt(1000 + cycle * 10);
  }
  ASSERT_FALSE(arm.published.empty());
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> errors;
  for (const auto & status : arm.published) {
    EXPECT_EQ(status.name, "fairino_hardware: ServoJ");
    EXPECT_EQ(status.hardware_id, "fairino10_v6");
    if (status.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
      errors.push_back(status);
    }
  }
  ASSERT_EQ(errors.size(), 3u) << "the onset, then 1 s and 2 s after it";
  EXPECT_EQ(errors[0].message, "ServoJ refused, code 14");
  EXPECT_EQ(valueOf(errors[0], "code"), "14");
  EXPECT_EQ(valueOf(errors[0], "refusals"), "1");
  EXPECT_EQ(valueOf(errors[0], "since_s"), "1.010");
  EXPECT_EQ(valueOf(errors[1], "refusals"), "101");
  EXPECT_EQ(valueOf(errors[2], "refusals"), "201");
  const auto & last = arm.published.back();
  EXPECT_EQ(last.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(last.message, "ServoJ accepted");
  // Before the episode, the OK was out at the first cycle.
  EXPECT_EQ(arm.published.front().level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(arm.published.size(), 5u) << "OK, the onset, two repeats, OK";
}

TEST(FairinoHardwareInterface, TheDriverNeverClearsAFault)
{
  ServoingDriver arm;
  arm.script->servo_codes = std::deque<int>(50, 14);
  for (int64_t cycle = 0; cycle < 60; ++cycle) {
    arm.cycleAt(cycle * 10);
  }
  for (size_t i = 2; i < arm.script->calls.size(); ++i) {
    const auto & call = arm.script->calls[i];
    EXPECT_TRUE(
      call == "ServoJ" || call == "GetActualJointPosDegree 1" ||
      call == "GetActualJointSpeedsDegree 1") << call;
  }
}

// The contract pinned on the topic, not only the sink: the component's own
// node publishes it on /diagnostics (domain 77, localhost, from CMake).
TEST(FairinoHardwareInterface, TheDiagnosticReachesTheTopic)
{
  rclcpp::init(0, nullptr);
  {
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto script = std::make_shared<SdkScript>();
    fairino_hardware::FairinoHardwareInterface driver(fakeSdk(script));
    hardware_interface::HardwareComponentParams params;
    params.hardware_info = robotWith(std::vector<std::vector<std::string>>(6, POSITION_ONLY));
    params.hardware_info.name = "fairino10_v6_hardware";
    params.clock = std::make_shared<rclcpp::Clock>();
    params.executor = executor;
    ASSERT_EQ(driver.init(params), CallbackReturn::SUCCESS);
    ASSERT_NE(driver.get_node(), nullptr);
    auto states = driver.export_state_interfaces();
    auto commands = driver.export_command_interfaces();
    ASSERT_EQ(driver.on_activate(INACTIVE), CallbackReturn::SUCCESS);

    auto listener = std::make_shared<rclcpp::Node>("diagnostic_listener");
    std::vector<diagnostic_msgs::msg::DiagnosticStatus> heard;
    auto subscription = listener->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10, [&heard](const diagnostic_msgs::msg::DiagnosticArray & array) {
        for (const auto & status : array.status) {
          heard.push_back(status);
        }
      });
    rclcpp::executors::SingleThreadedExecutor listening;
    listening.add_node(listener);

    script->servo_codes = std::deque<int>(1000, 14);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    bool refused = false;
    for (int64_t cycle = 0; !refused && std::chrono::steady_clock::now() < deadline; ++cycle) {
      const rclcpp::Time time(cycle * 10000000);
      driver.read(time, rclcpp::Duration(0, 10000000));
      driver.write(time, rclcpp::Duration(0, 10000000));
      listening.spin_some(std::chrono::milliseconds(10));
      for (const auto & status : heard) {
        refused = refused || (
          status.name == "fairino_hardware: ServoJ" &&
          status.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR);
      }
    }
    EXPECT_TRUE(refused) << heard.size() << " statuses heard in 2 s, none the refusal";
  }
  rclcpp::shutdown();
}

// Q-C1 (a), the owner's answer: once a refusal episode has begun, the
// controller executes whatever command is current the moment it accepts
// again. Until the command is back within resume_max_step_rad (0.01) of the
// measured position on every joint, the driver sends the measured position:
// a hold. It can only withhold motion.
TEST(FairinoHardwareInterface, AfterAnEpisodeTheDriverHoldsUntilTheCommandIsNearTheArm)
{
  ServoingDriver arm;
  LogCapture log;
  const double p2 = -20.0;  // j2, as the arm measures it (degrees)
  ASSERT_EQ(arm.cycleAt(0), hardware_interface::return_type::OK);
  arm.script->servo_codes = std::deque<int>(5, 14);  // 10 to 50 ms
  arm.command(1, p2 * DEG + 0.2);
  for (int64_t ms = 10; ms <= 50; ms += 10) {
    ASSERT_EQ(arm.cycleAt(ms), hardware_interface::return_type::OK);
  }
  ASSERT_EQ(arm.cycleAt(60), hardware_interface::return_type::OK);  // accepted again
  ASSERT_EQ(arm.cycleAt(70), hardware_interface::return_type::OK);
  const auto & accepted = arm.script->servo_degrees[6];
  EXPECT_NEAR(accepted[1], p2, 1e-9) << "the measured position, not the command 0.2 rad away";
  EXPECT_NEAR(arm.script->servo_degrees[7][1], p2, 1e-9);
  EXPECT_EQ(log.count(RCUTILS_LOG_SEVERITY_WARN, {"holding the arm"}), 1);

  arm.command(1, p2 * DEG + 0.005);  // within 0.01 rad of the arm
  ASSERT_EQ(arm.cycleAt(80), hardware_interface::return_type::OK);
  EXPECT_NEAR(arm.script->servo_degrees.back()[1], p2 + 0.005 / DEG, 1e-9) << "let go";
  arm.command(1, p2 * DEG + 0.2);  // no episode since: the command goes as it is
  ASSERT_EQ(arm.cycleAt(90), hardware_interface::return_type::OK);
  EXPECT_NEAR(arm.script->servo_degrees.back()[1], p2 + 0.2 / DEG, 1e-9);
}

TEST(FairinoHardwareInterface, TheResumeStepIsAHardwareParameter)
{
  auto script = std::make_shared<SdkScript>();
  auto info = robotWith(std::vector<std::vector<std::string>>(6, POSITION_ONLY));
  info.hardware_parameters["resume_max_step_rad"] = "-0.1";
  fairino_hardware::FairinoHardwareInterface refused(fakeSdk(script));
  EXPECT_EQ(refused.on_init(info), CallbackReturn::ERROR);
  info.hardware_parameters["resume_max_step_rad"] = "0.3";
  fairino_hardware::FairinoHardwareInterface wide(
    fakeSdk(script), [](const diagnostic_msgs::msg::DiagnosticStatus &) {return true;});
  ASSERT_EQ(wide.on_init(info), CallbackReturn::SUCCESS);
  auto commands = wide.export_command_interfaces();
  ASSERT_EQ(wide.on_activate(INACTIVE), CallbackReturn::SUCCESS);
  script->servo_codes = {14};
  (void)commands[1].set_value(-20.0 * DEG + 0.2);  // within 0.3 rad
  for (int64_t ms = 0; ms <= 20; ms += 10) {
    const rclcpp::Time time(ms * 1000000);
    ASSERT_EQ(wide.read(time, rclcpp::Duration(0, 10000000)), hardware_interface::return_type::OK);
    ASSERT_EQ(wide.write(time, rclcpp::Duration(0, 10000000)), hardware_interface::return_type::OK);
  }
  EXPECT_NEAR(script->servo_degrees.back()[1], -20.0 + 0.2 / DEG, 1e-9);
}
