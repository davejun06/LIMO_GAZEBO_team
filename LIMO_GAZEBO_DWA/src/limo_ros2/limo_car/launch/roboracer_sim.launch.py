#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    limo_share = get_package_share_directory("limo_car")
    gazebo_share = get_package_share_directory("gazebo_ros")

    default_world = os.path.join(limo_share, "worlds", "roboracer_obstacles_1", "ifac_roboracer_obstacles_1.world")
    default_rviz = os.path.join(limo_share, "rviz", "limo_view.rviz")

    world = LaunchConfiguration("world")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    use_rviz = LaunchConfiguration("rviz")

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(limo_share, "launch", "ackermann.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world, "verbose": "false"}.items(),
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "limo",
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", spawn_z,
            "-Y", spawn_yaw,
            "-timeout", "60",
            "--ros-args", "--log-level", "error",
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config"), "--ros-args", "--log-level", "error"],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("spawn_x", default_value="-0.0222"),
        # On the route with the whole body behind the start line; same as tools/exp/place_start.py.
        DeclareLaunchArgument("spawn_y", default_value="0.2205"),
        DeclareLaunchArgument("spawn_z", default_value="0.15"),
        DeclareLaunchArgument("spawn_yaw", default_value="-0.1375"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        robot_description,
        gazebo,
        TimerAction(period=2.0, actions=[spawn]),
        TimerAction(period=4.0, actions=[rviz]),
    ])
