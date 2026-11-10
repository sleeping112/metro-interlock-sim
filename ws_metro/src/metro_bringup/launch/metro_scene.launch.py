"""Full enriched scene: station, train, passengers, scenario and monitoring."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
            "headless": LaunchConfiguration("headless"),
            "rviz": LaunchConfiguration("rviz"),
            "auto_run": "true",
            "initial_x": LaunchConfiguration("initial_x"),
        }.items(),
    )

    scenario = Node(
        package="metro_application",
        executable="scenario_orchestrator",
        name="scenario_orchestrator",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "dwell_time_s": 20.0,
                "departure_speed_mps": 6.0,
                "departure_distance_m": 65.0,
                "open_timeout_s": 5.0,
            }
        ],
    )

    monitor = Node(
        package="metro_application",
        executable="platform_monitor",
        name="platform_monitor",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "psd_unit_count": 12,
                "station_name": "METRO CENTRAL",
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless", default_value="false",
                description="Run Gazebo without a GUI"),
            DeclareLaunchArgument(
                "rviz", default_value="true",
                description="Start RViz with the scene monitor"),
            DeclareLaunchArgument(
                "initial_x", default_value="-160.0",
                description="Train spawn position outside the station"),
            demo,
            scenario,
            monitor,
        ]
    )
