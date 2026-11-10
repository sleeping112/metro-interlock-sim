"""Start Gazebo Harmonic, spawn the train and bridge the configured Gazebo topics.

This launch intentionally contains no application logic.  The interlock and the
train speed controller live in metro_application and are started by
metro_demo.launch.py.
"""

import os

import xacro
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bridge_arguments(config_path):
    """Convert bridge.yaml entries to parameter_bridge CLI arguments.

    The CLI form is supported by every ROS 2 Humble ros_gz_bridge build, while
    the YAML file remains the canonical, reviewable configuration artifact.
    """
    with open(config_path, "r", encoding="utf-8") as stream:
        entries = yaml.safe_load(stream) or []

    arguments = []
    for entry in entries:
        ros_topic = entry["ros_topic_name"]
        gz_topic = entry["gz_topic_name"]
        if ros_topic != gz_topic:
            raise RuntimeError(
                "parameter_bridge CLI requires identical ROS and gz topic names; "
                f"got {ros_topic} vs {gz_topic}"
            )
        direction = str(entry.get("direction", "BIDIRECTIONAL")).upper()
        if direction == "GZ_TO_ROS":
            marker = "["
        elif direction == "ROS_TO_GZ":
            marker = "]"
        elif direction == "BIDIRECTIONAL":
            marker = "@"
        else:
            raise RuntimeError(f"Unsupported bridge direction: {direction}")
        arguments.append(
            f"{ros_topic}@{entry['ros_type_name']}{marker}{entry['gz_type_name']}"
        )
    return arguments


def _launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world").perform(context)
    initial_x = LaunchConfiguration("initial_x").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    rviz = LaunchConfiguration("rviz").perform(context).lower() == "true"

    bringup_share = get_package_share_directory("metro_bringup")
    description_share = get_package_share_directory("metro_description")
    description_prefix = get_package_prefix("metro_description")
    gazebo_prefix = get_package_prefix("metro_gazebo")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    model_path = os.path.join(description_prefix, "share", "metro_description", "models")
    plugin_path = os.path.join(gazebo_prefix, "lib")
    resource_paths = [model_path, os.path.join(bringup_share, "worlds")]
    if os.environ.get("GZ_SIM_RESOURCE_PATH"):
        resource_paths.append(os.environ["GZ_SIM_RESOURCE_PATH"])

    xacro_path = os.path.join(
        description_share, "models", "metro_train", "model.sdf.xacro"
    )
    if not os.path.isfile(xacro_path):
        raise RuntimeError(f"Train model not found: {xacro_path}")
    train_sdf = xacro.process_file(xacro_path).toxml()

    gz_args = f"-v 4 {'-s --headless-rendering ' if headless else ''}-r {world}"

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": gz_args,
            "on_exit_shutdown": "true",
        }.items(),
    )

    spawn_train = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_metro_train",
        output="screen",
        arguments=[
            "-world",
            "metro_station",
            "-name",
            "metro_train",
            "-string",
            train_sdf,
            "-x",
            initial_x,
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="metro_gz_bridge",
        output="screen",
        arguments=_bridge_arguments(os.path.join(bringup_share, "config", "bridge.yaml")),
        parameters=[
            {
                # /metro/train/odom is a 50 Hz sensor-data stream.  The bridge
                # normally creates a reliable publisher; override it to match
                # the contract.  Implementations that do not expose QoS
                # overrides silently ignore this parameter.
                "qos_overrides./metro/train/odom.publisher.reliability": "best_effort",
                "qos_overrides./metro/camera/platform/image.publisher.reliability": "best_effort",
                "qos_overrides./metro/camera/platform/camera_info.publisher.reliability": "best_effort",
            }
        ],
    )

    actions = [
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", os.pathsep.join(resource_paths)
        ),
        SetEnvironmentVariable(
            "GZ_SIM_SYSTEM_PLUGIN_PATH",
            os.pathsep.join(
                [plugin_path, os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")]
            ).strip(os.pathsep),
        ),
        gazebo,
        TimerAction(period=2.0, actions=[spawn_train, bridge]),
    ]

    if rviz:
        rviz_config = os.path.join(bringup_share, "rviz", "metro.rviz")
        actions.append(
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="metro_rviz",
                        output="screen",
                        arguments=["-d", rviz_config],
                        parameters=[{"use_sim_time": True}],
                    )
                ],
            )
        )

    return actions


def generate_launch_description():
    default_world = os.path.join(
        get_package_share_directory("metro_bringup"), "worlds", "metro_station.world"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Absolute path to the SDF world",
            ),
            DeclareLaunchArgument(
                "initial_x",
                default_value="-160.0",
                description="Initial longitudinal train position in metres",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run gz sim without the GUI (server only)",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz2 with the metro display config",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
