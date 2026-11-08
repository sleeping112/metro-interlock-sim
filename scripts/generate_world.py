#!/usr/bin/env python3
"""Authoring generator for metro_station.world (SDF 1.8).

The generated world is committed as a normal SDF file; this script is kept so
that the four repeated PSD units stay consistent.  It is not used at runtime.
"""

import os

OUT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "ws_metro", "src", "metro_bringup", "worlds", "metro_station.world",
)

UNIT_X = [-15.0, -5.0, 5.0, 15.0]


def frame_model(index, x):
    uid = f"{index:02d}"
    return f"""
    <!-- PSD frame unit {uid}: independent static model (no moving links). -->
    <model name="psd_unit_{uid}">
      <static>true</static>
      <pose>{x:.1f} 0 0 0 0 0</pose>
      <link name="psd_frame_{uid}">
        <collision name="psd_post_left_col_{uid}">
          <pose>-1.90 5.02 1.10 0 0 0</pose>
          <geometry><box><size>0.20 0.10 2.20</size></box></geometry>
        </collision>
        <collision name="psd_post_right_col_{uid}">
          <pose>1.90 5.02 1.10 0 0 0</pose>
          <geometry><box><size>0.20 0.10 2.20</size></box></geometry>
        </collision>
        <collision name="psd_header_col_{uid}">
          <pose>0 5.02 2.70 0 0 0</pose>
          <geometry><box><size>4.00 0.10 0.30</size></box></geometry>
        </collision>
        <collision name="psd_threshold_col_{uid}">
          <pose>0 5.02 0.325 0 0 0</pose>
          <geometry><box><size>4.00 0.10 0.05</size></box></geometry>
        </collision>

        <visual name="psd_post_left_vis_{uid}">
          <pose>-1.90 5.02 1.10 0 0 0</pose>
          <geometry><box><size>0.20 0.10 2.20</size></box></geometry>
          <material>
            <ambient>0.25 0.28 0.30 1</ambient>
            <diffuse>0.35 0.38 0.40 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
        <visual name="psd_post_right_vis_{uid}">
          <pose>1.90 5.02 1.10 0 0 0</pose>
          <geometry><box><size>0.20 0.10 2.20</size></box></geometry>
          <material>
            <ambient>0.25 0.28 0.30 1</ambient>
            <diffuse>0.35 0.38 0.40 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
        <visual name="psd_header_vis_{uid}">
          <pose>0 5.02 2.70 0 0 0</pose>
          <geometry><box><size>4.00 0.10 0.30</size></box></geometry>
          <material>
            <ambient>0.20 0.23 0.25 1</ambient>
            <diffuse>0.30 0.33 0.35 1</diffuse>
          </material>
        </visual>
        <visual name="psd_threshold_vis_{uid}">
          <pose>0 5.02 0.325 0 0 0</pose>
          <geometry><box><size>4.00 0.10 0.05</size></box></geometry>
          <material>
            <ambient>0.55 0.55 0.55 1</ambient>
            <diffuse>0.65 0.65 0.65 1</diffuse>
          </material>
        </visual>
        <visual name="psd_glass_left_vis_{uid}">
          <pose>-2.55 5.02 1.45 0 0 0</pose>
          <transparency>0.65</transparency>
          <geometry><box><size>0.90 0.05 2.05</size></box></geometry>
          <material>
            <ambient>0.55 0.75 0.80 0.35</ambient>
            <diffuse>0.65 0.85 0.90 0.35</diffuse>
          </material>
        </visual>
        <visual name="psd_glass_right_vis_{uid}">
          <pose>2.55 5.02 1.45 0 0 0</pose>
          <transparency>0.65</transparency>
          <geometry><box><size>0.90 0.05 2.05</size></box></geometry>
          <material>
            <ambient>0.55 0.75 0.80 0.35</ambient>
            <diffuse>0.65 0.85 0.90 0.35</diffuse>
          </material>
        </visual>
        <visual name="psd_unit_label_vis_{uid}">
          <pose>0 5.12 2.70 0 0 0</pose>
          <geometry><box><size>0.30 0.02 0.16</size></box></geometry>
          <material>
            <ambient>0.95 0.75 0.05 1</ambient>
            <diffuse>0.95 0.75 0.05 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""


def door_model(index, x):
    uid = f"{index:02d}"
    return f"""
    <!-- PSD sliding leaves unit {uid}; each leaf is an independent prismatic body. -->
    <model name="psd_door_{uid}">
      <static>false</static>
      <self_collide>false</self_collide>
      <allow_auto_disable>false</allow_auto_disable>
      <pose>{x:.1f} 0 0 0 0 0</pose>

      <link name="psd_base_{uid}">
        <pose>0 0 0 0 0 0</pose>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.001</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>0.001</iyy><iyz>0</iyz>
            <izz>0.001</izz>
          </inertia>
        </inertial>
      </link>

      <link name="psd_left_{uid}">
        <pose>-0.90 4.90 1.45 0 0 0</pose>
        <inertial>
          <mass>120.0</mass>
          <inertia>
            <ixx>48.436</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>80.800</iyy><iyz>0</iyz>
            <izz>32.436</izz>
          </inertia>
        </inertial>
        <collision name="psd_left_col_{uid}">
          <geometry><box><size>1.80 0.06 2.20</size></box></geometry>
          <surface>
            <friction><ode><mu>0.05</mu><mu2>0.05</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="psd_left_vis_{uid}">
          <transparency>0.60</transparency>
          <geometry><box><size>1.80 0.06 2.20</size></box></geometry>
          <material>
            <ambient>0.55 0.78 0.85 0.40</ambient>
            <diffuse>0.65 0.88 0.95 0.40</diffuse>
          </material>
        </visual>
      </link>

      <link name="psd_right_{uid}">
        <pose>0.90 4.90 1.45 0 0 0</pose>
        <inertial>
          <mass>120.0</mass>
          <inertia>
            <ixx>48.436</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>80.800</iyy><iyz>0</iyz>
            <izz>32.436</izz>
          </inertia>
        </inertial>
        <collision name="psd_right_col_{uid}">
          <geometry><box><size>1.80 0.06 2.20</size></box></geometry>
          <surface>
            <friction><ode><mu>0.05</mu><mu2>0.05</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="psd_right_vis_{uid}">
          <transparency>0.60</transparency>
          <geometry><box><size>1.80 0.06 2.20</size></box></geometry>
          <material>
            <ambient>0.55 0.78 0.85 0.40</ambient>
            <diffuse>0.65 0.88 0.95 0.40</diffuse>
          </material>
        </visual>
      </link>

      <joint name="psd_base_fixed_{uid}" type="fixed">
        <parent>world</parent>
        <child>psd_base_{uid}</child>
      </joint>

      <joint name="psd_left_joint_{uid}" type="prismatic">
        <parent>psd_base_{uid}</parent>
        <child>psd_left_{uid}</child>
        <axis>
          <xyz>1 0 0</xyz>
          <limit>
            <lower>-1.80</lower>
            <upper>0.0</upper>
            <effort>5000.0</effort>
            <velocity>1.0</velocity>
          </limit>
          <dynamics><damping>2.0</damping><friction>0.2</friction></dynamics>
        </axis>
      </joint>

      <joint name="psd_right_joint_{uid}" type="prismatic">
        <parent>psd_base_{uid}</parent>
        <child>psd_right_{uid}</child>
        <axis>
          <xyz>1 0 0</xyz>
          <limit>
            <lower>0.0</lower>
            <upper>1.80</upper>
            <effort>5000.0</effort>
            <velocity>1.0</velocity>
          </limit>
          <dynamics><damping>2.0</damping><friction>0.2</friction></dynamics>
        </axis>
      </joint>
    </model>
"""


def build_world():
    psd_frames = "".join(frame_model(i + 1, x) for i, x in enumerate(UNIT_X))
    psd_doors = "".join(door_model(i + 1, x) for i, x in enumerate(UNIT_X))
    return f"""<?xml version="1.0" ?>
<!--
  Single-line metro station world.

  Coordinates: x along the track, y lateral (platform centred at y=0), z up.
  Platform top is z=0.30.  The train runs on the +y track bed (centre y=6.65).
  Four PSD units are centred at x=-15,-5,+5,+15 with a 1.8 m sliding-leaf
  travel.  The DoorController system plugin is loaded as a world plugin.
-->
<sdf version="1.8">
  <world name="metro_station">
    <gravity>0 0 -9.8</gravity>
    <physics name="metro_physics" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>100</real_time_update_rate>
      <dart>
        <solver><solver_type>dantzig</solver_type></solver>
        <collision_detector>bullet</collision_detector>
      </dart>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>

    <scene>
      <ambient>0.55 0.58 0.62 1</ambient>
      <background>0.75 0.82 0.90 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="station_sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.95 0.95 0.90 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.35 0.25 -0.90</direction>
    </light>
    <light type="point" name="platform_fill">
      <pose>0 0 8 0 0 0</pose>
      <diffuse>0.65 0.70 0.78 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation>
        <range>50</range>
        <constant>0.6</constant>
        <linear>0.02</linear>
        <quadratic>0.0</quadratic>
      </attenuation>
    </light>

    <!-- Static station platform: 50 x 10 x 0.3 m, top at z=0.30. -->
    <model name="station_platform">
      <static>true</static>
      <link name="platform_link">
        <collision name="platform_collision">
          <pose>0 0 0.15 0 0 0</pose>
          <geometry><box><size>50.0 10.0 0.30</size></box></geometry>
        </collision>
        <visual name="platform_top_visual">
          <pose>0 0 0.15 0 0 0</pose>
          <geometry><box><size>50.0 10.0 0.30</size></box></geometry>
          <material>
            <ambient>0.55 0.55 0.52 1</ambient>
            <diffuse>0.68 0.68 0.64 1</diffuse>
          </material>
        </visual>
        <visual name="safety_line_visual">
          <pose>0 4.85 0.305 0 0 0</pose>
          <geometry><box><size>50.0 0.10 0.01</size></box></geometry>
          <material>
            <ambient>0.95 0.75 0.05 1</ambient>
            <diffuse>0.95 0.75 0.05 1</diffuse>
          </material>
        </visual>
        <visual name="tactile_strip_visual">
          <pose>0 4.55 0.305 0 0 0</pose>
          <geometry><box><size>50.0 0.35 0.01</size></box></geometry>
          <material>
            <ambient>0.80 0.65 0.10 1</ambient>
            <diffuse>0.85 0.70 0.12 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Both track beds: 50 x 2 x 0.2 m, static, top at z=0.20. -->
    <model name="track_bed_positive_y">
      <static>true</static>
      <link name="track_link">
        <collision name="track_collision">
          <pose>0 6.65 0.10 0 0 0</pose>
          <geometry><box><size>50.0 2.0 0.20</size></box></geometry>
        </collision>
        <visual name="track_ballast_visual">
          <pose>0 6.65 0.10 0 0 0</pose>
          <geometry><box><size>50.0 2.0 0.20</size></box></geometry>
          <material>
            <ambient>0.18 0.18 0.18 1</ambient>
            <diffuse>0.24 0.24 0.24 1</diffuse>
          </material>
        </visual>
        <visual name="rail_positive_y_outer">
          <pose>0 7.35 0.24 0 0 0</pose>
          <geometry><box><size>50.0 0.08 0.08</size></box></geometry>
          <material>
            <ambient>0.30 0.32 0.35 1</ambient>
            <diffuse>0.45 0.47 0.50 1</diffuse>
          </material>
        </visual>
        <visual name="rail_positive_y_inner">
          <pose>0 5.95 0.24 0 0 0</pose>
          <geometry><box><size>50.0 0.08 0.08</size></box></geometry>
          <material>
            <ambient>0.30 0.32 0.35 1</ambient>
            <diffuse>0.45 0.47 0.50 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="track_bed_negative_y">
      <static>true</static>
      <link name="track_link">
        <collision name="track_collision">
          <pose>0 -6.65 0.10 0 0 0</pose>
          <geometry><box><size>50.0 2.0 0.20</size></box></geometry>
        </collision>
        <visual name="track_ballast_visual">
          <pose>0 -6.65 0.10 0 0 0</pose>
          <geometry><box><size>50.0 2.0 0.20</size></box></geometry>
          <material>
            <ambient>0.18 0.18 0.18 1</ambient>
            <diffuse>0.24 0.24 0.24 1</diffuse>
          </material>
        </visual>
        <visual name="rail_negative_y_outer">
          <pose>0 -7.35 0.24 0 0 0</pose>
          <geometry><box><size>50.0 0.08 0.08</size></box></geometry>
          <material>
            <ambient>0.30 0.32 0.35 1</ambient>
            <diffuse>0.45 0.47 0.50 1</diffuse>
          </material>
        </visual>
        <visual name="rail_negative_y_inner">
          <pose>0 -5.95 0.24 0 0 0</pose>
          <geometry><box><size>50.0 0.08 0.08</size></box></geometry>
          <material>
            <ambient>0.30 0.32 0.35 1</ambient>
            <diffuse>0.45 0.47 0.50 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
{psd_frames}
{psd_doors}
    <!-- World-level executor for all four PSD units. -->
    <plugin filename="libmetro_door_controller.so" name="metro::gazebo::DoorController">
      <unit_count>4</unit_count>
      <unit_model_prefix>psd_door_</unit_model_prefix>
      <left_joint_prefix>psd_left_joint_</left_joint_prefix>
      <right_joint_prefix>psd_right_joint_</right_joint_prefix>
      <door_speed_mps>0.8</door_speed_mps>
      <door_travel_m>1.8</door_travel_m>
      <state_rate_hz>10.0</state_rate_hz>
      <lock_delay_s>0.3</lock_delay_s>
      <open_position_tolerance_m>0.01</open_position_tolerance_m>
    </plugin>
  </world>
</sdf>
"""


def main():
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as stream:
        stream.write(build_world())
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
