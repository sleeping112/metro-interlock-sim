#!/usr/bin/env python3
"""Static self-check for the metro simulation workspace.

Usage:
    python3 scripts/verify_workspace.py
"""

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC = ROOT / "ws_metro" / "src"

PACKAGES = [
    "metro_description",
    "metro_gazebo",
    "metro_application",
    "metro_bringup",
]

REQUIRED_FILES = [
    "README.md",
    "docs/stage1_design.md",
    "docs/acceptance.md",
    "ws_metro/src/metro_description/models/metro_train/model.sdf.xacro",
    "ws_metro/src/metro_description/models/metro_psd_unit/model.sdf.xacro",
    "ws_metro/src/metro_description/urdf/metro_train.urdf.xacro",
    "ws_metro/src/metro_description/meshes/unit_box.stl",
    "ws_metro/src/metro_description/models/metro_train/meshes/unit_box.stl",
    "ws_metro/src/metro_description/models/metro_station/textures/station_name.png",
    "ws_metro/src/metro_gazebo/src/door_controller.cpp",
    "ws_metro/src/metro_gazebo/src/train_longitudinal.cpp",
    "ws_metro/src/metro_gazebo/src/passenger_flow.cpp",
    "ws_metro/src/metro_gazebo/src/ambient_train.cpp",
    "ws_metro/src/metro_gazebo/src/intrusion_detector.cpp",
    "ws_metro/src/metro_application/src/interlock_node.cpp",
    "ws_metro/src/metro_application/src/train_speed_controller_node.cpp",
    "ws_metro/src/metro_application/src/scenario_orchestrator.cpp",
    "ws_metro/src/metro_application/src/platform_monitor.cpp",
    "ws_metro/src/metro_bringup/worlds/metro_station.world",
    "ws_metro/src/metro_bringup/launch/metro_demo.launch.py",
    "ws_metro/src/metro_bringup/launch/metro_scene.launch.py",
    "ws_metro/src/metro_bringup/launch/acceptance.launch.py",
    "ws_metro/src/metro_bringup/config/bridge.yaml",
    "ws_metro/src/metro_bringup/scripts/acceptance_test.py",
]

FAULTS = [
    "TRAIN_NOT_IN_STOP_WINDOW",
    "PSD_NOT_LOCKED",
    "PSD_OPEN_TIMEOUT",
    "EMERGENCY_BRAKE_PSD_OPEN",
]


def check(condition, description, errors):
    if condition:
        print(f"ok   {description}")
    else:
        print(f"FAIL {description}")
        errors.append(description)


def read(path):
    return path.read_text(encoding="utf-8", errors="replace")


def main():
    errors = []

    for rel in REQUIRED_FILES:
        check((ROOT / rel).is_file(), f"file exists: {rel}", errors)

    for pkg in PACKAGES:
        pkg_dir = SRC / pkg
        check(pkg_dir.is_dir(), f"package directory: {pkg}", errors)
        check((pkg_dir / "package.xml").is_file(), f"package.xml: {pkg}", errors)
        check((pkg_dir / "CMakeLists.txt").is_file(), f"CMakeLists.txt: {pkg}", errors)

    world = read(SRC / "metro_bringup/worlds/metro_station.world")
    check('version="1.8"' in world, "world is SDF 1.8", errors)
    check("<max_step_size>0.001</max_step_size>" in world, "world step 0.001 s", errors)
    check("<real_time_update_rate>100</real_time_update_rate>" in world, "world 100 Hz", errors)
    for index in range(1, 13):
        uid = f"{index:02d}"
        check(f'name="psd_unit_{uid}"' in world, f"static frame psd_unit_{uid}", errors)
        check(f'name="psd_door_{uid}"' in world, f"door model psd_door_{uid}", errors)
        check(f'name="psd_left_{uid}"' in world, f"left leaf psd_left_{uid}", errors)
        check(f'name="psd_right_{uid}"' in world, f"right leaf psd_right_{uid}", errors)
        check(f'name="psd_left_joint_{uid}"' in world, f"left joint {uid}", errors)
        check(f'name="psd_right_joint_{uid}"' in world, f"right joint {uid}", errors)
    check(world.count("<static>true</static>") >= 30, "static platform/track/frame models", errors)
    check("130.0 14.0 0.3" in world or "130.0 14.0 0.30" in world, "130 x 14 m platform", errors)
    for token in ["passenger_01", "ambient_train", "platform_camera",
                  "libmetro_passenger_flow.so", "libmetro_intrusion_detector.so",
                  "gz-sim-sensors-system"]:
        check(token in world, f"enriched world contains {token}", errors)

    train = read(SRC / "metro_description/models/metro_train/model.sdf.xacro")
    for token in ["120.0", "3.0", "3.8", "30000.0", "train_longitudinal",
                  "train_door_collision_01", "train_door_collision_12",
                  "car_06_body"]:
        check(token in train, f"train model contains {token}", errors)
    check("libmetro_train_plugin.so" in train, "train plugin reference", errors)

    interlock = read(SRC / "metro_application/src/interlock_node.cpp")
    for fault in FAULTS:
        check(fault in interlock, f"interlock implements {fault}", errors)
    check("SensorDataQoS" in interlock, "interlock uses sensor-data QoS for odom", errors)
    check("get_clock()->now()" in interlock, "interlock uses simulated clock", errors)

    application = "\n".join(
        read(path) for path in (SRC / "metro_application/src").glob("*.cpp")
    )
    check("std::this_thread::sleep_for" not in application, "no sleep_for in nodes", errors)
    check("system_clock" not in application, "no system_clock in nodes", errors)

    plugins = "\n".join(
        read(path) for path in (SRC / "metro_gazebo/src").glob("*.cpp")
    )
    check("GZ_ADD_PLUGIN" in plugins, "gz plugin registration present", errors)
    check('"/metro/psd/unit_"' in plugins and "setw(2)" in plugins,
          "per-unit service names generated with two-digit index", errors)

    bridge = read(SRC / "metro_bringup/config/bridge.yaml")
    for topic in ["/clock", "/metro/train/odom", "/metro/train/command",
                  "/metro/camera/platform/image"]:
        check(topic in bridge, f"bridge entry for {topic}", errors)
    check("sensor_data" in bridge, "odom/camera bridge QoS profile documented", errors)
    for topic in ["/metro/platform/intrusion", "/metro/monitor/markers"]:
        check(topic in "".join(read(path) for path in (SRC / "metro_application/src").glob("*.cpp")),
              f"monitor topic {topic}", errors)

    params = read(SRC / "metro_bringup/config/interlock_params.yaml")
    for key in ["stop_window_m", "psd_open_timeout_s", "emergency_speed_threshold_mps",
                "fault_hold_s", "fault_repeat_suppression_s", "auto_run"]:
        check(key in params, f"parameter default: {key}", errors)

    launch = read(SRC / "metro_bringup/launch/metro_sim.launch.py")
    for token in ["GZ_SIM_RESOURCE_PATH", "GZ_SIM_SYSTEM_PLUGIN_PATH",
                  "ros_gz_sim", "parameter_bridge", "qos_overrides",
                  "--headless-rendering"]:
        check(token in launch, f"launch contains {token}", errors)

    print()
    if errors:
        print(f"FAILED: {len(errors)} problem(s)")
        return 1
    print("OK: workspace static checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
