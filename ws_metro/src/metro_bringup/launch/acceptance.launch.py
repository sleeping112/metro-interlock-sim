"""Automated end-to-end acceptance launch.

Starts the whole stack in headless mode with the automatic speed curve disabled,
then runs the black-box acceptance tester against the public interface.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    Shutdown,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("metro_bringup")
    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, "launch", "metro_demo.launch.py")
        ),
        launch_arguments={
            "headless": "true",
            "rviz": "false",
            "auto_run": "false",
            "initial_x": LaunchConfiguration("initial_x"),
        }.items(),
    )

    tester = Node(
        package="metro_bringup",
        executable="acceptance_test.py",
        name="metro_acceptance_tester",
        output="screen",
        on_exit=Shutdown(reason="acceptance scenario finished"),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "initial_x",
                default_value="-80.0",
                description="Initial train x (outside the stop window)",
            ),
            demo,
            TimerAction(period=4.0, actions=[tester]),
        ]
    )
