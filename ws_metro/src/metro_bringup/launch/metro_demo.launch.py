"""End-to-end demo: simulator, bridges, interlock and optional RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("metro_bringup")
    sim_launch = os.path.join(share, "launch", "metro_sim.launch.py")
    interlock_params = os.path.join(share, "config", "interlock_params.yaml")

    initial_x = LaunchConfiguration("initial_x")
    headless = LaunchConfiguration("headless")
    rviz = LaunchConfiguration("rviz")
    auto_run = LaunchConfiguration("auto_run")

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            "initial_x": initial_x,
            "headless": headless,
            "rviz": rviz,
        }.items(),
    )

    interlock = Node(
        package="metro_application",
        executable="interlock_node",
        name="interlock_node",
        output="screen",
        parameters=[
            interlock_params,
            {
                "use_sim_time": True,
                "auto_run": ParameterValue(auto_run, value_type=bool),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "initial_x",
                default_value="-80.0",
                description="Initial train x position (must match spawn)",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo without GUI",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz2",
            ),
            DeclareLaunchArgument(
                "auto_run",
                default_value="true",
                description="Use the automatic station-stop speed curve",
            ),
            simulator,
            interlock,
        ]
    )
