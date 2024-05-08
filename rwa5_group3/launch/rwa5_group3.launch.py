import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction
)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ariac_moveit_config.parameters import generate_parameters


def launch_setup(context, *args, **kwargs):
    floor_robot_node = Node(
        package="rwa5_group3",
        executable="rwa5_group3_cpp_exe",
        output="screen",
        parameters=generate_parameters(),
    )

    yolo_python_node = Node(package="rwa5_group3", executable="yolo.py", output="screen")

    start_rviz = LaunchConfiguration("rviz")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rwa5_group3"), "rviz", "rwa5_group3.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=generate_parameters(),
        condition=IfCondition(start_rviz),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("ariac_moveit_config"),
                "/launch",
                "/ariac_robots_moveit.launch.py",
            ]
        )
    )
    timer_action_moveit = TimerAction(
        period = 5.0,
        actions=[moveit]
    )
    timer_action_rviz = TimerAction(
        period = 5.0,
        actions=[rviz_node]
    )
    timer_action_yolo = TimerAction(
        period = 15.0,
        actions=[yolo_python_node]
    )
    timer_action_floor = TimerAction(
        period = 20.0,
        actions=[floor_robot_node]
    )
    nodes_to_start = [
        timer_action_moveit,
        timer_action_rviz,
        timer_action_yolo,
        timer_action_floor
    ]


    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz", default_value="false", description="start rviz node?"
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )