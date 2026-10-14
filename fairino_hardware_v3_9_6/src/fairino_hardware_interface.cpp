#include "fairino_hardware/fairino_hardware_interface.hpp"
#include "libfairino/include/robot_error.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>

namespace fairino_hardware{

FairinoHardwareInterface::FairinoHardwareInterface()
: _sdk_factory(makeRobotSdk)
{
}

FairinoHardwareInterface::FairinoHardwareInterface(RobotSdkFactory sdk_factory)
: _sdk_factory(std::move(sdk_factory))
{
}

FairinoHardwareInterface::FairinoHardwareInterface(
    RobotSdkFactory sdk_factory, DiagnosticSink diagnostic_sink)
: _sdk_factory(std::move(sdk_factory)), _diagnostic_sink(std::move(diagnostic_sink))
{
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_init(const hardware_interface::HardwareInfo& sysinfo){
    if (hardware_interface::SystemInterface::on_init(sysinfo) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    info_ = sysinfo;//info_是父类中定义的变量
    
    for (const hardware_interface::ComponentInfo& joint : info_.joints) {

        //指令部分
        if (joint.command_interfaces.size() != 1) {//开放servoJ
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                        joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // if (joint.command_interfaces[1].name != hardware_interface::HW_IF_EFFORT){//预留，用于关节扭矩直接控制
        //     RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
        //            "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
        //            joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_EFFORT);
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

        //关节状态部分
        // F-204: position, then optionally velocity -- the robot-measured
        // joint speed. Without it joint_state_broadcaster publishes every
        // velocity as NaN, which MoveIt Servo takes in when it re-seeds
        // from /joint_states and turns into NaN commands (F-184).
        // F-447: then optionally effort -- the robot-measured joint torque;
        // without it every /joint_states effort is NaN, which pymoveit2
        // 5.0.0 refuses in a start state.
        if (joint.state_interfaces.size() < 1 || joint.state_interfaces.size() > 3) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"), "Joint '%s' has %zu state interfaces. 1 to 3 expected.",
                        joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() >= 2 &&
            joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Joint '%s' have %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
                        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() == 3 &&
            joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Joint '%s' have %s state interface as third state interface. '%s' expected.", joint.name.c_str(),
                        joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
            return hardware_interface::CallbackReturn::ERROR;
        }

    }
    // Every joint or none: a velocity on some joints only is refused here
    // rather than published half NaN.
    const size_t with_velocity = std::count_if(info_.joints.begin(), info_.joints.end(),
        [](const hardware_interface::ComponentInfo& joint) { return joint.state_interfaces.size() >= 2; });
    if (with_velocity != 0 && with_velocity != info_.joints.size()) {
        RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                    "%zu of %zu joints declare a velocity state interface: all or none expected.",
                    with_velocity, info_.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }
    _export_velocity = with_velocity != 0;
    const size_t with_effort = std::count_if(info_.joints.begin(), info_.joints.end(),
        [](const hardware_interface::ComponentInfo& joint) { return joint.state_interfaces.size() == 3; });
    if (with_effort != 0 && with_effort != info_.joints.size()) {
        RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                    "%zu of %zu joints declare an effort state interface: all or none expected.",
                    with_effort, info_.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }
    _export_effort = with_effort != 0;

    const auto tolerance = info_.hardware_parameters.find("transient_read_error_tolerance_ms");
    if (tolerance != info_.hardware_parameters.end()) {
        int64_t ms = -1;
        try {
            size_t used = 0;
            ms = std::stoll(tolerance->second, &used);
            if (used != tolerance->second.size()) {
                ms = -1;
            }
        } catch (const std::exception &) {
            ms = -1;
        }
        if (ms < 0) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                        "transient_read_error_tolerance_ms: '%s' is not a count of milliseconds",
                        tolerance->second.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        _transient_read_error_tolerance_ns = ms * 1000000;
    }

    const auto drill = info_.hardware_parameters.find("servoj_refusal_injection");
    if (drill != info_.hardware_parameters.end() && drill->second != "false") {
        if (drill->second != "true") {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                        "servoj_refusal_injection: '%s' is neither true nor false", drill->second.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        const char * allowed = std::getenv(SERVOJ_REFUSAL_INJECTION_ENV);
        if (allowed == nullptr || allowed[0] == '\0' || allowed[0] == '0') {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                        "servoj_refusal_injection is true, but %s is not set: the code-14 and "
                        "read-error drills are bench drills and are refused elsewhere",
                        SERVOJ_REFUSAL_INJECTION_ENV);
            return hardware_interface::CallbackReturn::ERROR;
        }
        _injection_available = true;
        RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                    "servoj_refusal_injection is on: ~/inject_servoj_refusal (std_srvs/SetBool) makes "
                    "write() send no ServoJ and read code 14, the arm still; "
                    "~/inject_read_errors_under and ~/inject_read_errors_over (std_srvs/Trigger) make "
                    "the next %d or %d state reads return -2. Bench drills, never a production run",
                    readErrorsUnderTheBound(), readErrorsOverTheBound());
    }

    const auto step = info_.hardware_parameters.find("resume_max_step_rad");
    if (step != info_.hardware_parameters.end()) {
        double rad = -1.0;
        try {
            size_t used = 0;
            rad = std::stod(step->second, &used);
            if (used != step->second.size()) {
                rad = -1.0;
            }
        } catch (const std::exception &) {
            rad = -1.0;
        }
        if (!(rad > 0.0) || !std::isfinite(rad)) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                        "resume_max_step_rad: '%s' is not a positive angle in radians",
                        step->second.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        _resume_max_step_rad = rad;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}//end on_init



std::vector<hardware_interface::StateInterface> FairinoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  //导出关节相关的状态接口(位置，速度，扭矩)
  for (size_t i = 0; i < info_.joints.size(); ++i){
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_state[i]));

    if (_export_velocity) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_jnt_velocity_state[i]));
    }

    if (_export_effort) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_state[i]));
    }
  }

  //导出
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FairinoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_command[i]));

//     command_interfaces.emplace_back(hardware_interface::CommandInterface(//预留的扭矩控制接口
//         info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_command.at(i)));
  }

  return command_interfaces;
}



hardware_interface::CallbackReturn FairinoHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    using namespace std::chrono_literals;
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Starting ...please wait...");
    //做变量的初始化工作
    try {
        _ptr_robot = _sdk_factory();//创建机器人实例
    } catch (const std::exception & error) {
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "No robot SDK: %s", error.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    for(int i=0;i<6;i++){//初始化变量
        _jnt_position_command[i] = 0;
        _jnt_velocity_command[i] = 0;
        _jnt_torque_command[i] = 0;
        _jnt_position_state[i] = 0;
        _jnt_velocity_state[i] = 0;
        _jnt_torque_state[i] = 0;
    }
    _read_failing = false;
    _refusal = RefusalEpisode();
    _resume_hold = false;
    _hold_said = false;
    _injecting = false;
    _read_errors_owed = 0;
    if (_injection_available && !_injection_service && get_node() != nullptr) {
        _injection_service = get_node()->create_service<std_srvs::srv::SetBool>(
            "~/inject_servoj_refusal",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                response->success = injectServoJRefusal(request->data);
                response->message = request->data ? "ServoJ refusal drill on" : "ServoJ refusal drill off";
            });
    }
    if (_injection_available && !_read_errors_under_service && get_node() != nullptr) {
        auto arm = [this](int reads) {
                return [this, reads](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                    response->success = injectReadErrors(reads);
                    response->message = std::to_string(reads) + " state reads of -2 armed";
                };
            };
        _read_errors_under_service = get_node()->create_service<std_srvs::srv::Trigger>(
            "~/inject_read_errors_under", arm(readErrorsUnderTheBound()));
        _read_errors_over_service = get_node()->create_service<std_srvs::srv::Trigger>(
            "~/inject_read_errors_over", arm(readErrorsOverTheBound()));
    }
    if (!_diagnostic_sink) {
        // F-226: the contract on /diagnostics, from the component's own node
        // (ros2_control creates it); a RealtimePublisher, whose try-lock never
        // waits in the control loop.
        if (get_node() != nullptr) {
            _diagnostic_publisher = std::make_shared<
                realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>(
                get_node()->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                    "/diagnostics", rclcpp::SystemDefaultsQoS()));
            auto publisher = _diagnostic_publisher;
            auto node = get_node();
            _diagnostic_sink = [publisher, node](const diagnostic_msgs::msg::DiagnosticStatus & status) {
                diagnostic_msgs::msg::DiagnosticArray array;
                array.header.stamp = node->now();
                array.status.push_back(status);
                return publisher->try_publish(array);
            };
        } else {
            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                        "no component node: the ServoJ diagnostic is not published");
        }
    }
    _control_mode = 0;//默认是位置控制,0-位置控制，1-扭矩控制 2-速度控制
    errno_t returncode = _ptr_robot->RPC(_controller_ip.c_str());//建立xmlrpc连接
    rclcpp::sleep_for(200ms);//等待一段时间让控制器的rpc连接建立完毕
    if(returncode != 0){
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "机械臂SDK连接失败！请检查端口时候被占用");
        return hardware_interface::CallbackReturn::ERROR;
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "机械臂SDK连接成功！");
    }
    //做第一步的工作，读取当前状态数据
    JointPos jntpos;
    returncode = _ptr_robot->GetActualJointPosDegree(0,&jntpos);
    /*
    获取反馈位置后同步到指令位置以维持当前状态，如果发现读取失败，那么就无法激活插件，
    因为错误的反馈位置会导致初始指令位置下发出现严重偏差导致事故
    */
    if(returncode == 0){
        for(int j=0;j<6;j++){
            _jnt_position_command[j] = jntpos.jPos[j]/180.0*M_PI;
        }
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),"初始指令位置: %f,%f,%f,%f,%f,%f",_jnt_position_command[0],\
        _jnt_position_command[1],_jnt_position_command[2],_jnt_position_command[3],_jnt_position_command[4],_jnt_position_command[5]);    
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "机械臂硬件启动成功!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "读取初始关节角度错误，硬件无法启动！请检查通讯内容");
        return hardware_interface::CallbackReturn::ERROR;
    }
}



hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Stopping ...please wait...");
    _ptr_robot->StopMotion();//停止机器人
    _ptr_robot->CloseRPC();//销毁实例，连接断开
    // F-417: reset(), not release(): release() dropped the SDK object
    // without deleting it, one leaked per deactivation.
    _ptr_robot.reset();
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "System successfully stopped!");
    return hardware_interface::CallbackReturn::SUCCESS;
}



bool FairinoHardwareInterface::acceptReadCode(errno_t code, const char * what, const rclcpp::Time & time)
{
    if (code != ERR_SOCKET_COM_FAILED) {
        // F-204: a failed read used to go on as OK and republish the old
        // positions; it fails the cycle.
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "%s failed: %d", what, code);
        return false;
    }
    if (!_read_failing) {
        _read_failing = true;
        _read_failing_since = time;
        _read_failures = 0;
        RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                    "%s failed: %d, the last good state held (up to %ld ms)", what, code,
                    static_cast<long>(_transient_read_error_tolerance_ns / 1000000));
    }
    const int64_t failing_ns = (time - _read_failing_since).nanoseconds();
    if (failing_ns > _transient_read_error_tolerance_ns) {
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                     "%s failed: %d, for %ld ms (past the %ld ms bound)", what, code,
                     static_cast<long>(failing_ns / 1000000),
                     static_cast<long>(_transient_read_error_tolerance_ns / 1000000));
        return false;
    }
    return true;
}

hardware_interface::return_type FairinoHardwareInterface::read(const rclcpp::Time& time,const rclcpp::Duration& period)
{//从RTDE反馈数据中获取所需的位置，速度和扭矩信息
    bool transient = false;
    // The read-error drill: this cycle's reads answer -2 without the SDK.
    int owed = _read_errors_owed.load();
    while (owed > 0 && !_read_errors_owed.compare_exchange_weak(owed, owed - 1)) {
    }
    const bool drilled = owed > 0;
    JointPos state_data;
    error_t returncode = drilled ? ERR_SOCKET_COM_FAILED : _ptr_robot->GetActualJointPosDegree(1,&state_data);
    if(returncode == 0){
        for(int i=0;i<6;i++){
            _jnt_position_state[i] = state_data.jPos[i]/180.0*M_PI;//注意单位转换，moveit统一用弧度
            //_jnt_torque_state[i] = state_data.jt_cur_tor[i];//注意单位转换
        }
    }else if (acceptReadCode(returncode, "GetActualJointPosDegree", time)) {
        transient = true;  // F-323: the last good positions, held
    }else{
        return hardware_interface::return_type::ERROR;
    }
    if (_export_velocity) {
        // The robot's own feedback (actual_qd in the real-time state, flag 1:
        // non-blocking, the same packet as the positions), never a finite
        // difference of positions.
        float speed[6];
        returncode = drilled ? ERR_SOCKET_COM_FAILED : _ptr_robot->GetActualJointSpeedsDegree(1, speed);
        if (returncode == 0) {
            for (int i = 0; i < 6; i++) {
                if (!std::isfinite(speed[i])) {
                    RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "joint %d speed is not finite", i + 1);
                    return hardware_interface::return_type::ERROR;
                }
                _jnt_velocity_state[i] = speed[i] / 180.0 * M_PI;
            }
        } else if (acceptReadCode(returncode, "GetActualJointSpeedsDegree", time)) {
            transient = true;  // F-323: the last good speeds, held
        } else {
            return hardware_interface::return_type::ERROR;
        }
    }
    if (_export_effort) {
        // F-447: the robot-measured joint torque (flag 1: non-blocking, from
        // the real-time state as the speeds are), as the SDK gives it.
        float torque[6];
        returncode = drilled ? ERR_SOCKET_COM_FAILED : _ptr_robot->GetJointTorques(1, torque);
        if (returncode == 0) {
            for (int i = 0; i < 6; i++) {
                if (!std::isfinite(torque[i])) {
                    RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "joint %d torque is not finite", i + 1);
                    return hardware_interface::return_type::ERROR;
                }
                _jnt_torque_state[i] = torque[i];
            }
        } else if (acceptReadCode(returncode, "GetJointTorques", time)) {
            transient = true;  // F-323: the last good torques, held
        } else {
            return hardware_interface::return_type::ERROR;
        }
    }
    if (transient) {
        ++_read_failures;
    } else if (_read_failing) {
        RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                    "read recovered after %d failures over %ld ms", _read_failures,
                    static_cast<long>((time - _read_failing_since).nanoseconds() / 1000000));
        _read_failing = false;
    }
    //RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "System successfully read: %f,%f,%f,%f,%f,%f",_jnt_position_state[0],\
    _jnt_position_state[1],_jnt_position_state[2],_jnt_position_state[3],_jnt_position_state[4],_jnt_position_state[5]);

  return hardware_interface::return_type::OK;

}

bool FairinoHardwareInterface::injectServoJRefusal(bool on)
{
    if (!_injection_available) {
        return false;
    }
    _injecting = on;
    RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                "ServoJ refusal drill %s", on ? "ON: no ServoJ is sent, every cycle reads as code 14"
                                              : "off: ServoJ is sent again");
    return true;
}

bool FairinoHardwareInterface::injectReadErrors(int reads)
{
    if (!_injection_available || reads < 1) {
        return false;
    }
    _read_errors_owed = reads;
    RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                "read-error drill: the next %d state reads return -2 (the bound is %ld ms)", reads,
                static_cast<long>(_transient_read_error_tolerance_ns / 1000000));
    return true;
}

int FairinoHardwareInterface::readErrorsUnderTheBound() const
{
    return std::max<int64_t>(1, _transient_read_error_tolerance_ns / 20000000);
}

int FairinoHardwareInterface::readErrorsOverTheBound() const
{
    return std::max<int64_t>(2, _transient_read_error_tolerance_ns / 5000000);
}

diagnostic_msgs::msg::DiagnosticStatus FairinoHardwareInterface::servoJStatus() const
{
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "fairino_hardware: ServoJ";
    status.hardware_id = "fairino10_v6";
    if (_refusal.active()) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "ServoJ refused, code " + std::to_string(_refusal.code());
        char since[32];
        std::snprintf(since, sizeof(since), "%.3f", _refusal.sinceNs() / 1e9);
        diagnostic_msgs::msg::KeyValue value;
        value.key = "code";
        value.value = std::to_string(_refusal.code());
        status.values.push_back(value);
        value.key = "refusals";
        value.value = std::to_string(_refusal.refusals());
        status.values.push_back(value);
        value.key = "since_s";
        value.value = since;
        status.values.push_back(value);
    } else {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "ServoJ accepted";
    }
    return status;
}

void FairinoHardwareInterface::noteServoJ(int code, const rclcpp::Time & time)
{
    const auto event = _refusal.observe(code, time.nanoseconds());
    if (event == RefusalEpisode::Event::Started) {
        _resume_hold = true;
    }
    if (event == RefusalEpisode::Event::Started || event == RefusalEpisode::Event::CodeChanged) {
        // The vendor's own text kept beside the English: every row and sheet
        // that greps 错误码 still finds the onset.
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                     "ServoJ refused, code %d (ServoJ指令下发错误,错误码:%d); the arm is not following commands",
                     code, code);
    } else if (event == RefusalEpisode::Event::Ended) {
        RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                    "ServoJ accepted again after %d refusals over %.2f s (code %d)",
                    _refusal.refusals(), (_refusal.endedNs() - _refusal.sinceNs()) / 1e9,
                    _refusal.code());
    }
    if (event == RefusalEpisode::Event::None) {
        return;
    }
    if (!_diagnostic_sink || _diagnostic_sink(servoJStatus())) {
        _refusal.said(time.nanoseconds());
    } else {
        _refusal.unsaid();  // the publisher was busy: due again next cycle
    }
}

hardware_interface::return_type FairinoHardwareInterface::write(const rclcpp::Time& time,const rclcpp::Duration& period)
{
    if(_control_mode == 0){//位置控制模式
        if (std::any_of(&_jnt_position_command[0], &_jnt_position_command[6],\
            [](double c) { return not std::isfinite(c); })) {
            return hardware_interface::return_type::ERROR;
        }
        JointPos cmd;
        ExaxisPos extcmd{0,0,0,0};
        for(auto j=0;j<6;j++){
            cmd.jPos[j] = _jnt_position_command[j]/M_PI*180; //注意单位转换
        }
        if (_resume_hold) {
            double farthest = 0.0;
            for (int j = 0; j < 6; j++) {
                farthest = std::max(farthest, std::abs(_jnt_position_command[j] - _jnt_position_state[j]));
            }
            if (farthest > _resume_max_step_rad) {
                if (!_hold_said) {
                    RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                                "holding the arm: the command is %.3f rad from it after a refusal episode",
                                farthest);
                    _hold_said = true;
                }
                for (int j = 0; j < 6; j++) {
                    cmd.jPos[j] = _jnt_position_state[j]/M_PI*180;
                }
            } else if (!_refusal.active()) {
                _resume_hold = false;
                _hold_said = false;
            }
        }
        //RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "ServoJ下发位置:%f,%f,%f,%f,%f,%f",\
            cmd.jPos[0],cmd.jPos[1],cmd.jPos[2],cmd.jPos[3],cmd.jPos[4],cmd.jPos[5]);
        // The code-14 drill: nothing reaches the controller, and the cycle
        // reads as the refusal the bench has met.
        int returncode = _injecting ? ERR_EXECUTION_FAILED : _ptr_robot->ServoJ(&cmd,&extcmd,0,0,0.008,0,0);
        // F-226: this was an INFO line every 10 ms, and nothing else; the
        // refusal is now an episode, said once and published.
        noteServoJ(returncode, time);
    }else if(_control_mode == 1){//扭矩控制模式
        if (std::any_of(&_jnt_torque_command[0], &_jnt_torque_command[6],\
            [](double c) { return not std::isfinite(c); })) {
            return hardware_interface::return_type::ERROR;
        }
        //_ptr_robot->write(_jnt_torque_command);//注意单位转换
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "指令发送错误:未识别当前所处控制模式");
        return hardware_interface::return_type::ERROR;
    }
 
    return hardware_interface::return_type::OK;
}


}//end namesapce

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fairino_hardware::FairinoHardwareInterface, hardware_interface::SystemInterface)
