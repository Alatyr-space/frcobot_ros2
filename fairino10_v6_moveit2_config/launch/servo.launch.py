#!/usr/bin/env python3
"""Launch MoveIt Servo for Fairino FR10 with startup diagnostics.
Needs the MoveIt2 demo launch to be running first, e.g.:
  ros2 launch fairino10_v6_moveit2_config demo.launch.py
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


PACKAGE_NAME = "fairino10_v6_moveit2_config"
ROBOT_NAME = "fairino10_v6_robot"


def load_yaml(package_name, relative_file_path):
    package_share_directory = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_share_directory, relative_file_path)

    with open(absolute_file_path, "r", encoding="utf-8") as yaml_file:
        return yaml.safe_load(yaml_file)


def yaml_get(data, key, default="NOT FOUND"):
    return data.get(key, default) if isinstance(data, dict) else default


# The parameters MoveIt Servo 2.12.4 declares (Alatyr F-346). Source:
# moveit_servo/config/servo_parameters.yaml at the 2.12.4 tag, cross-checked
# against the 40 `moveit_servo.*` lines of a Servo node's own parameter dump;
# nested ones are written flattened (`scale.linear`).
SERVO_2_12_4_PARAMETERS = frozenset([
    "active_subgroup",
    "apply_twist_commands_about_ee_frame",
    "cartesian_command_in_topic",
    "check_collisions",
    "check_octomap_collisions",
    "collision_check_rate",
    "command_in_type",
    "command_out_topic",
    "command_out_type",
    "halt_all_joints_in_cartesian_mode",
    "halt_all_joints_in_joint_mode",
    "hard_stop_singularity_threshold",
    "incoming_command_timeout",
    "is_primary_planning_scene_monitor",
    "joint_command_in_topic",
    "joint_limit_margins",
    "joint_topic",
    "leaving_singularity_threshold_multiplier",
    "lower_singularity_threshold",
    "max_expected_latency",
    "monitored_planning_scene_topic",
    "move_group_name",
    "override_velocity_scaling_factor",
    "pose_command_in_topic",
    "pose_tracking.angular_tolerance",
    "pose_tracking.linear_tolerance",
    "publish_joint_accelerations",
    "publish_joint_positions",
    "publish_joint_velocities",
    "publish_period",
    "scale.joint",
    "scale.linear",
    "scale.rotational",
    "scene_collision_proximity_threshold",
    "self_collision_proximity_threshold",
    "singularity_step_scale",
    "smoothing_filter_plugin_name",
    "status_topic",
    "thread_priority",
    "use_smoothing",
])

# Launch-only YAML keys that Servo does not declare but this launch reads.
# None today: move_group_name is a Servo parameter and also feeds
# planning_group_name below.
LAUNCH_ONLY_KEYS = frozenset()


def flatten_keys(mapping, prefix=""):
    names = []
    for key, value in mapping.items():
        if isinstance(value, dict):
            names.extend(flatten_keys(value, f"{prefix}{key}."))
        else:
            names.append(f"{prefix}{key}")
    return names


def print_servo_diagnostics(servo_yaml, servo_log_level):
    keys = [
        "publish_period",
        "incoming_command_timeout",
        "command_in_type",
        "scale",
        "move_group_name",
        "cartesian_command_in_topic",
        "joint_topic",
        "status_topic",
        "command_out_type",
        "command_out_topic",
        "publish_joint_positions",
        "publish_joint_velocities",
        "publish_joint_accelerations",
        "check_collisions",
        "self_collision_proximity_threshold",
        "scene_collision_proximity_threshold",
        "lower_singularity_threshold",
        "hard_stop_singularity_threshold",
        "joint_limit_margins",
        "apply_twist_commands_about_ee_frame",
    ]

    print("[servo.launch.py] ===== MoveIt Servo diagnostic summary =====")
    print(f"[servo.launch.py] package={PACKAGE_NAME}, robot={ROBOT_NAME}, servo_log_level={servo_log_level}")
    for key in keys:
        print(f"[servo.launch.py] {key}: {yaml_get(servo_yaml, key)}")
    # A key Servo ignores must not look as if it were in force (Alatyr F-346).
    if isinstance(servo_yaml, dict):
        for key in flatten_keys(servo_yaml):
            if key not in SERVO_2_12_4_PARAMETERS and key not in LAUNCH_ONLY_KEYS:
                print(f"[servo.launch.py] WARNING: {key} is not a MoveIt Servo 2.12.4 parameter and is ignored")
    print("[servo.launch.py] Expected command input topic with node name 'servo_node': /servo_node/delta_twist_cmds")
    print("[servo.launch.py] Expected status topic with node name 'servo_node': /servo_node/status")
    print("[servo.launch.py] Expected controller output topic: " + str(yaml_get(servo_yaml, "command_out_topic")))
    print("[servo.launch.py] If Servo prints 'Waiting to receive robot state update', check joint_topic freshness and joint names.")
    print("[servo.launch.py] =============================================")


def launch_setup(context, *args, **kwargs):
    moveit_config = (
        MoveItConfigsBuilder(ROBOT_NAME, package_name=PACKAGE_NAME)
        .to_moveit_configs()
    )

    servo_config_file = LaunchConfiguration("servo_config_file").perform(context)
    servo_log_level = LaunchConfiguration("servo_log_level").perform(context)
    if servo_config_file:
        with open(servo_config_file, "r", encoding="utf-8") as yaml_file:
            servo_yaml = yaml.safe_load(yaml_file)
        print(f"[servo.launch.py] Loaded Servo YAML from override: {servo_config_file}")
    else:
        servo_yaml = load_yaml(PACKAGE_NAME, "config/servo_parameters.yaml")
        print("[servo.launch.py] Loaded Servo YAML from package config/servo_parameters.yaml")

    print_servo_diagnostics(servo_yaml, servo_log_level)

    servo_params = {"moveit_servo": servo_yaml}

    publish_period = float(yaml_get(servo_yaml, "publish_period", 0.01))
    acceleration_filter_update_period = {"update_period": publish_period}
    planning_group_name = {"planning_group_name": yaml_get(servo_yaml, "move_group_name", "fairino10_v6_group")}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        # No `name=`: launch_ros turns it into a process-wide
        # `-r __node:=servo_node`, which renames every node the process
        # creates, and MoveIt Servo creates two, so the graph held two
        # `/servo_node` (Alatyr F-274). Servo names its main node
        # `servo_node` itself, so `/servo_node/pause_servo` and the other
        # services keep their names.
        output="screen",
        arguments=["--ros-args", "--log-level", servo_log_level],
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return [servo_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "servo_config_file",
                default_value="",
                description=(
                    "Optional absolute path to a Servo YAML file. "
                    "If empty, config/servo_parameters.yaml from this package is used."
                ),
            ),
            DeclareLaunchArgument(
                "servo_start_delay",
                default_value="5.0",
                description="Delay in seconds before starting Servo.",
            ),
            DeclareLaunchArgument(
                "servo_log_level",
                default_value="info",
                description="ROS log level for moveit_servo servo_node. Use debug while diagnosing.",
            ),
            TimerAction(
                period=LaunchConfiguration("servo_start_delay"),
                actions=[OpaqueFunction(function=launch_setup)],
            ),
        ]
    )
