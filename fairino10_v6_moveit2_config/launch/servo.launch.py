#!/usr/bin/env python3
"""Launch the Fairino FR10 MoveIt demo stack, then MoveIt Servo."""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
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


def launch_setup(context, *args, **kwargs):
    moveit_config = (
        MoveItConfigsBuilder(ROBOT_NAME, package_name=PACKAGE_NAME)
        .to_moveit_configs()
    )

    servo_config_file = LaunchConfiguration("servo_config_file").perform(context)
    if servo_config_file:
        with open(servo_config_file, "r", encoding="utf-8") as yaml_file:
            servo_yaml = yaml.safe_load(yaml_file)
    else:
        servo_yaml = load_yaml(PACKAGE_NAME, "config/servo_parameters.yaml")

    servo_params = {"moveit_servo": servo_yaml}

    # Used by the default online_signal_smoothing::AccelerationLimitedPlugin.
    # Keep this equal to publish_period in servo_parameters.yaml.
    acceleration_filter_update_period = {"update_period": 0.01}

    # Used by the acceleration limiting filter.
    planning_group_name = {"planning_group_name": "fairino10_v6_group"}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="screen",
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
    demo_launch = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        "launch",
        "demo.launch.py",
    )

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
                description=(
                    "Delay in seconds before starting Servo after demo.launch.py is included."
                ),
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(demo_launch)),
            TimerAction(
                period=LaunchConfiguration("servo_start_delay"),
                actions=[OpaqueFunction(function=launch_setup)],
            ),
        ]
    )
