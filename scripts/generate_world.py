#!/usr/bin/env python3
"""Authoring generator for the enriched metro_station.world (SDF 1.8).

Coordinates
-----------
x : along the track (station centre at 0)
y : lateral, platform spans [-7, +7] m, train track centre at +8.65 m
z : up, platform top at 0.30 m

The generated world is committed as a normal SDF file.  This script keeps the
12 repeated PSD units, 6 train cars, passengers, station furniture and tunnel
sections consistent.  It is not used at runtime.
"""

import os

OUT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "ws_metro", "src", "metro_bringup", "worlds", "metro_station.world",
)

PLATFORM_LENGTH = 130.0
PLATFORM_WIDTH = 14.0
PLATFORM_TOP = 0.30
PLATFORM_EDGE_Y = 7.0
TRACK_Y = 8.65
TRACK_LENGTH = 180.0
UNIT_COUNT = 12
UNIT_X = [-55.0 + 10.0 * i for i in range(UNIT_COUNT)]
TRAIN_SPAWN_X = -160.0


def material(ambient, diffuse, emissive=None, specular=None):
    parts = [
        "<material>",
        f"<ambient>{ambient}</ambient>",
        f"<diffuse>{diffuse}</diffuse>",
    ]
    if specular is not None:
        parts.append(f"<specular>{specular}</specular>")
    if emissive is not None:
        parts.append(f"<emissive>{emissive}</emissive>")
    parts.append("</material>")
    return "".join(parts)


def texture_material(texture):
    return (
        "<material><pbr><metal>"
        f"<albedo_map>model://metro_station/textures/{texture}</albedo_map>"
        "<roughness>0.65</roughness><metalness>0.05</metalness>"
        "</metal></pbr></material>"
    )


def visual(name, pose, size, mat, transparency=None):
    tr = f"<transparency>{transparency}</transparency>" if transparency else ""
    return (
        f'<visual name="{name}"><pose>{pose}</pose>{tr}'
        f"<geometry><box><size>{size}</size></box></geometry>{mat}</visual>"
    )


def collision(name, pose, size):
    return (
        f'<collision name="{name}"><pose>{pose}</pose>'
        f"<geometry><box><size>{size}</size></box></geometry></collision>"
    )


def frame_model(index, x):
    uid = f"{index:02d}"
    steel = material("0.22 0.25 0.28 1", "0.30 0.34 0.38 1", specular="0.35 0.35 0.35 1")
    dark = material("0.10 0.12 0.14 1", "0.16 0.18 0.20 1")
    glass = material("0.55 0.75 0.82 1", "0.65 0.85 0.92 1")
    yellow = material("0.95 0.72 0.04 1", "0.98 0.78 0.08 1",
                      emissive="0.35 0.25 0.02 1")
    return f"""
    <!-- PSD frame unit {uid}: independent static model (no moving links). -->
    <model name="psd_unit_{uid}">
      <static>true</static>
      <pose>{x:.1f} 0 0 0 0 0</pose>
      <link name="psd_frame_{uid}">
        {collision(f"psd_post_left_col_{uid}", "-1.90 7.02 1.10 0 0 0", "0.20 0.10 2.20")}
        {collision(f"psd_post_right_col_{uid}", "1.90 7.02 1.10 0 0 0", "0.20 0.10 2.20")}
        {collision(f"psd_header_col_{uid}", "0 7.02 2.70 0 0 0", "4.00 0.10 0.30")}
        {collision(f"psd_threshold_col_{uid}", "0 7.02 0.325 0 0 0", "4.00 0.10 0.05")}
        {visual(f"psd_post_left_vis_{uid}", "-1.90 7.02 1.10 0 0 0", "0.20 0.10 2.20", steel)}
        {visual(f"psd_post_right_vis_{uid}", "1.90 7.02 1.10 0 0 0", "0.20 0.10 2.20", steel)}
        {visual(f"psd_header_vis_{uid}", "0 7.02 2.70 0 0 0", "4.00 0.10 0.30", dark)}
        {visual(f"psd_threshold_vis_{uid}", "0 7.02 0.325 0 0 0", "4.00 0.10 0.05", steel)}
        {visual(f"psd_glass_left_vis_{uid}", "-2.55 7.02 1.45 0 0 0", "0.90 0.05 2.05", glass, 0.65)}
        {visual(f"psd_glass_right_vis_{uid}", "2.55 7.02 1.45 0 0 0", "0.90 0.05 2.05", glass, 0.65)}
        {visual(f"psd_unit_label_vis_{uid}", "0 7.10 2.70 0 0 0", "0.30 0.02 0.16", yellow)}
        {visual(f"psd_upper_panel_vis_{uid}", "0 7.02 2.46 0 0 0", "3.60 0.06 0.18", steel)}
      </link>
    </model>
"""


def door_model(index, x):
    uid = f"{index:02d}"
    glass = material("0.55 0.78 0.85 1", "0.65 0.88 0.95 1")
    edge = material("0.15 0.18 0.20 1", "0.22 0.25 0.28 1")
    return f"""
    <!-- PSD sliding leaves unit {uid}; each leaf is an independent prismatic body. -->
    <model name="psd_door_{uid}">
      <static>false</static>
      <self_collide>false</self_collide>
      <allow_auto_disable>false</allow_auto_disable>
      <pose>{x:.1f} 0 0 0 0 0</pose>

      <link name="psd_base_{uid}">
        <pose>0 0 0 0 0 0</pose>
        <inertial><mass>1.0</mass><inertia>
          <ixx>0.001</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.001</iyy><iyz>0</iyz><izz>0.001</izz>
        </inertia></inertial>
      </link>

      <link name="psd_left_{uid}">
        <pose>-0.90 6.90 1.45 0 0 0</pose>
        <inertial><mass>120.0</mass><inertia>
          <ixx>48.436</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>80.800</iyy><iyz>0</iyz><izz>32.436</izz>
        </inertia></inertial>
        <collision name="psd_left_col_{uid}"><geometry><box><size>1.80 0.06 2.20</size></box></geometry></collision>
        {visual(f"psd_left_vis_{uid}", "0 0 0 0 0 0", "1.80 0.06 2.20", glass, 0.60)}
        {visual(f"psd_left_edge_vis_{uid}", "0.86 0 0 0 0 0", "0.03 0.08 2.20", edge)}
      </link>

      <link name="psd_right_{uid}">
        <pose>0.90 6.90 1.45 0 0 0</pose>
        <inertial><mass>120.0</mass><inertia>
          <ixx>48.436</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>80.800</iyy><iyz>0</iyz><izz>32.436</izz>
        </inertia></inertial>
        <collision name="psd_right_col_{uid}"><geometry><box><size>1.80 0.06 2.20</size></box></geometry></collision>
        {visual(f"psd_right_vis_{uid}", "0 0 0 0 0 0", "1.80 0.06 2.20", glass, 0.60)}
        {visual(f"psd_right_edge_vis_{uid}", "-0.86 0 0 0 0 0", "0.03 0.08 2.20", edge)}
      </link>

      <joint name="psd_base_fixed_{uid}" type="fixed">
        <parent>world</parent><child>psd_base_{uid}</child>
      </joint>
      <joint name="psd_left_joint_{uid}" type="prismatic">
        <parent>psd_base_{uid}</parent><child>psd_left_{uid}</child>
        <axis><xyz>1 0 0</xyz><limit>
          <lower>-1.80</lower><upper>0.0</upper>
          <effort>5000.0</effort><velocity>1.0</velocity>
        </limit><dynamics><damping>2.0</damping><friction>0.2</friction></dynamics></axis>
      </joint>
      <joint name="psd_right_joint_{uid}" type="prismatic">
        <parent>psd_base_{uid}</parent><child>psd_right_{uid}</child>
        <axis><xyz>1 0 0</xyz><limit>
          <lower>0.0</lower><upper>1.80</upper>
          <effort>5000.0</effort><velocity>1.0</velocity>
        </limit><dynamics><damping>2.0</damping><friction>0.2</friction></dynamics></axis>
      </joint>
    </model>
"""


def platform_model():
    tile = material("0.50 0.52 0.50 1", "0.66 0.67 0.63 1")
    edge = material("0.38 0.39 0.38 1", "0.50 0.51 0.50 1")
    yellow = material("0.96 0.72 0.04 1", "0.98 0.78 0.08 1")
    tactile = material("0.82 0.62 0.06 1", "0.88 0.68 0.10 1")
    visuals = [
        visual("platform_top_visual", "0 0 0.15 0 0 0",
               f"{PLATFORM_LENGTH} {PLATFORM_WIDTH} {PLATFORM_TOP}", tile),
        visual("platform_front_edge", f"0 {PLATFORM_EDGE_Y - 0.05} 0.31 0 0 0",
               f"{PLATFORM_LENGTH} 0.10 0.02", edge),
        visual("safety_line_visual", f"0 {PLATFORM_EDGE_Y - 0.15} 0.305 0 0 0",
               f"{PLATFORM_LENGTH} 0.10 0.01", yellow),
        visual("tactile_strip_visual", f"0 {PLATFORM_EDGE_Y - 0.45} 0.305 0 0 0",
               f"{PLATFORM_LENGTH} 0.35 0.01", tactile),
    ]
    for i, y in enumerate([-5.5, -3.0, -0.5, 2.0, 4.5]):
        visuals.append(
            visual(f"platform_inlay_{i}", f"0 {y} 0.305 0 0 0",
                   f"{PLATFORM_LENGTH} 0.04 0.008",
                   material("0.35 0.38 0.40 1", "0.45 0.48 0.50 1")))
    for i, x in enumerate([-60, -40, -20, 0, 20, 40, 60]):
        visuals.append(
            visual(f"platform_tile_band_{i}", f"{x} 0 0.305 0 0 0",
                   "0.06 14.0 0.008",
                   material("0.42 0.44 0.43 1", "0.55 0.57 0.55 1")))
    return f"""
    <!-- Island platform: 130 x 14 x 0.3 m, top at z=0.30. -->
    <model name="station_platform">
      <static>true</static>
      <link name="platform_link">
        {collision("platform_collision", "0 0 0.15 0 0 0",
                   f"{PLATFORM_LENGTH} {PLATFORM_WIDTH} {PLATFORM_TOP}")}
        {''.join(visuals)}
      </link>
    </model>
"""


def track_model(y, suffix):
    ballast = material("0.16 0.16 0.16 1", "0.23 0.23 0.23 1")
    sleeper = material("0.24 0.20 0.16 1", "0.32 0.27 0.21 1")
    rail = material("0.34 0.36 0.39 1", "0.50 0.52 0.55 1",
                    specular="0.8 0.8 0.8 1")
    parts = [
        collision(f"track_collision_{suffix}", f"0 {y} 0.10 0 0 0",
                  f"{TRACK_LENGTH} 2.0 0.20"),
        visual(f"track_ballast_{suffix}", f"0 {y} 0.10 0 0 0",
               f"{TRACK_LENGTH} 2.0 0.20", ballast),
        visual(f"rail_inner_{suffix}", f"0 {y - 0.70} 0.24 0 0 0",
               f"{TRACK_LENGTH} 0.08 0.08", rail),
        visual(f"rail_outer_{suffix}", f"0 {y + 0.70} 0.24 0 0 0",
               f"{TRACK_LENGTH} 0.08 0.08", rail),
    ]
    for i in range(30):
        x = -87.0 + i * 6.0
        parts.append(
            visual(f"sleeper_{suffix}_{i:02d}", f"{x:.1f} {y} 0.245 0 0 0",
                   "0.28 2.15 0.08", sleeper))
    return f"""
    <model name="track_bed_{suffix}">
      <static>true</static>
      <link name="track_link_{suffix}">
        {''.join(parts)}
      </link>
    </model>
"""


def station_structure():
    concrete = material("0.55 0.57 0.59 1", "0.68 0.70 0.72 1")
    ceiling = material("0.70 0.72 0.74 1", "0.82 0.84 0.86 1")
    column = material("0.60 0.62 0.64 1", "0.74 0.76 0.78 1")
    beam = material("0.42 0.44 0.46 1", "0.55 0.57 0.59 1")
    light = material("1.0 0.95 0.80 1", "1.0 0.97 0.86 1",
                     emissive="1.0 0.92 0.72 1")
    parts = [
        visual("station_ceiling", "0 0 5.48 0 0 0",
               f"{PLATFORM_LENGTH} {PLATFORM_WIDTH} 0.20", ceiling),
    ]
    for i, x in enumerate([-60, -50, -40, -30, -20, -10, 0, 10, 20, 30, 40, 50, 60]):
        parts.append(
            visual(f"roof_beam_{i:02d}", f"{x} 0 5.18 0 0 0",
                   f"0.35 {PLATFORM_WIDTH} 0.25", beam))
        for side in (-1, 1):
            parts.append(
                visual(f"column_{i:02d}_{'p' if side > 0 else 'n'}",
                       f"{x} {side * 4.8} 2.80 0 0 0",
                       "0.35 0.35 5.00", column))
    for i, y in enumerate([-5.5, -2.75, 0.0, 2.75, 5.5]):
        parts.append(
            visual(f"light_strip_{i}", f"0 {y} 5.36 0 0 0",
                   f"{PLATFORM_LENGTH - 2.0} 0.10 0.035", light))
    return f"""
    <model name="station_structure">
      <static>true</static>
      <link name="structure_link">
        {''.join(parts)}
      </link>
    </model>
"""


def furniture():
    wood = material("0.35 0.22 0.12 1", "0.48 0.30 0.16 1")
    metal = material("0.25 0.27 0.29 1", "0.35 0.37 0.40 1")
    plastic = material("0.12 0.14 0.16 1", "0.18 0.20 0.23 1")
    green = material("0.05 0.35 0.18 1", "0.08 0.45 0.24 1")
    red = material("0.65 0.08 0.06 1", "0.80 0.10 0.08 1")
    parts = []
    for i, x in enumerate([-45, -25, -5, 15, 35, 55]):
        parts.append(visual(f"bench_seat_{i}", f"{x} -4.0 0.78 0 0 0",
                            "3.0 0.62 0.12", wood))
        parts.append(visual(f"bench_back_{i}", f"{x} -4.28 1.18 0 0 0",
                            "3.0 0.10 0.70", wood))
        for j, dx in enumerate((-1.25, 1.25)):
            parts.append(visual(f"bench_leg_{i}_{j}", f"{x + dx} -4.0 0.48 0 0 0",
                                "0.10 0.55 0.36", metal))
    for i, x in enumerate([-55, -35, -15, 5, 25, 45, 60]):
        parts.append(visual(f"bin_{i}", f"{x} -6.0 0.75 0 0 0",
                            "0.55 0.55 0.90", plastic))
        parts.append(visual(f"bin_lid_{i}", f"{x} -6.0 1.23 0 0 0",
                            "0.60 0.60 0.06", metal))
    for i, x in enumerate([-52, -32, -12, 8, 28, 48]):
        parts.append(visual(f"ad_box_{i}", f"{x} 6.55 2.1 0 0 0",
                            "2.0 0.12 1.4",
                            material("0.92 0.92 0.88 1", "1.0 1.0 0.96 1",
                                     emissive="0.25 0.25 0.23 1")))
    # Ticket gate row near the west end of the platform.
    for i, y in enumerate((-4.8, -2.4, 0.0, 2.4, 4.8)):
        parts.append(visual(f"gate_pillar_{i}", f"-62 {y} 1.05 0 0 0",
                            "0.55 0.90 1.50", metal))
        parts.append(visual(f"gate_light_{i}", f"-62 {y} 1.72 0 0 0",
                            "0.18 0.18 0.18", green if i % 2 == 0 else red))
    parts.append(visual("ticket_machine_a", "-59 -5.2 1.05 0 0 0",
                        "1.0 0.70 1.50", metal))
    parts.append(visual("ticket_machine_b", "-59 -3.8 1.05 0 0 0",
                        "1.0 0.70 1.50", metal))
    parts.append(visual("fire_cabinet", "58 -6.2 1.0 0 0 0",
                        "0.80 0.45 0.90", red))
    parts.append(visual("service_desk", "-56 5.2 1.05 0 0 0",
                        "4.0 0.80 1.20", metal))
    return f"""
    <model name="station_furniture">
      <static>true</static>
      <link name="furniture_link">
        {''.join(parts)}
      </link>
    </model>
"""


def signage():
    dark = material("0.05 0.08 0.12 1", "0.08 0.12 0.18 1")
    parts = [
        visual("station_name_board", f"0 -6.82 4.05 0 0 0",
               "8.0 0.08 1.2", texture_material("station_name.png")),
        visual("line_map_west", f"-38 -6.82 3.55 0 0 0",
               "4.0 0.08 2.0", texture_material("line_map.png")),
        visual("line_map_east", f"38 -6.82 3.55 0 0 0",
               "4.0 0.08 2.0", texture_material("line_map.png")),
        visual("exit_board_west", f"-61 -6.82 3.1 0 0 0",
               "2.8 0.08 0.7", texture_material("exit_sign.png")),
        visual("mind_gap_board_01", f"-25 7.12 2.95 0 0 0",
               "3.2 0.06 0.45", texture_material("mind_the_gap.png")),
        visual("mind_gap_board_02", f"25 7.12 2.95 0 0 0",
               "3.2 0.06 0.45", texture_material("mind_the_gap.png")),
    ]
    for i, x in enumerate([-52, -32, -12, 8, 28, 48]):
        parts.append(visual(f"ad_frame_{i}", f"{x} 6.55 2.1 0 0 0",
                            "2.1 0.16 1.5", dark))
    return f"""
    <model name="station_signage">
      <static>true</static>
      <link name="signage_link">
        {''.join(parts)}
      </link>
    </model>
"""


def tunnel_model(y, suffix):
    dark = material("0.05 0.05 0.06 1", "0.09 0.09 0.10 1")
    concrete = material("0.24 0.25 0.26 1", "0.33 0.34 0.35 1")
    parts = []
    for x in (-85.0, 85.0):
        parts += [
            visual(f"tunnel_floor_{suffix}_{'w' if x < 0 else 'e'}",
                   f"{x} {y} 0.10 0 0 0", "40.0 4.0 0.20", concrete),
            visual(f"tunnel_ceiling_{suffix}_{'w' if x < 0 else 'e'}",
                   f"{x} {y} 4.20 0 0 0", "40.0 4.0 0.20", dark),
            visual(f"tunnel_wall_inner_{suffix}_{'w' if x < 0 else 'e'}",
                   f"{x} {y - 1.9} 2.10 0 0 0", "40.0 0.20 4.0", concrete),
            visual(f"tunnel_wall_outer_{suffix}_{'w' if x < 0 else 'e'}",
                   f"{x} {y + 1.9} 2.10 0 0 0", "40.0 0.20 4.0", concrete),
        ]
    return f"""
    <model name="tunnel_{suffix}">
      <static>true</static>
      <link name="tunnel_link_{suffix}">
        {''.join(parts)}
      </link>
    </model>
"""


def end_walls():
    concrete = material("0.38 0.40 0.42 1", "0.50 0.52 0.54 1")
    dark = material("0.04 0.04 0.05 1", "0.07 0.07 0.08 1")
    parts = []
    for x in (-65.2, 65.2):
        parts.append(visual(f"end_wall_{'w' if x < 0 else 'e'}",
                            f"{x} 0 2.85 0 0 0", "0.35 14.0 5.10", concrete))
        parts.append(visual(f"end_wall_trim_{'w' if x < 0 else 'e'}",
                            f"{x - 0.20 if x < 0 else x + 0.20} 0 2.85 0 0 0",
                            "0.06 14.0 5.10", dark))
    return f"""
    <model name="station_end_walls">
      <static>true</static>
      <link name="end_wall_link">
        {''.join(parts)}
      </link>
    </model>
"""


def catenary():
    metal = material("0.26 0.27 0.28 1", "0.36 0.37 0.38 1")
    wire = material("0.12 0.12 0.12 1", "0.20 0.20 0.20 1")
    parts = []
    for i, x in enumerate(range(-80, 81, 20)):
        for y in (-10.7, 10.7):
            parts.append(visual(f"catenary_pole_{i}_{'p' if y > 0 else 'n'}",
                                f"{x} {y} 3.6 0 0 0", "0.12 0.12 6.2", metal))
        parts.append(visual(f"catenary_cross_{i}", f"{x} 0 6.55 0 0 0",
                            "0.10 21.5 0.10", metal))
    for y in (-8.65, 8.65):
        parts.append(visual(f"contact_wire_{'p' if y > 0 else 'n'}",
                            f"0 {y} 6.0 0 0 0", "180.0 0.025 0.025", wire))
    return f"""
    <model name="overhead_line">
      <static>true</static>
      <link name="overhead_link">
        {''.join(parts)}
      </link>
    </model>
"""


def passenger_model(index):
    uid = f"{index:02d}"
    x = -54.0 + 9.0 * (index - 1)
    y = -2.4 + 0.55 * ((index * 7) % 5)
    coat_colors = [
        ("0.10 0.22 0.42 1", "0.14 0.30 0.56 1"),
        ("0.48 0.12 0.10 1", "0.62 0.16 0.13 1"),
        ("0.16 0.34 0.22 1", "0.20 0.44 0.28 1"),
        ("0.40 0.32 0.10 1", "0.52 0.42 0.14 1"),
        ("0.18 0.18 0.20 1", "0.25 0.25 0.28 1"),
        ("0.34 0.18 0.42 1", "0.44 0.24 0.54 1"),
    ]
    amb, dif = coat_colors[(index - 1) % len(coat_colors)]
    coat = material(amb, dif)
    skin = material("0.55 0.38 0.26 1", "0.68 0.48 0.33 1")
    trousers = material("0.10 0.12 0.16 1", "0.16 0.18 0.22 1")
    hair = material("0.06 0.05 0.04 1", "0.10 0.08 0.06 1")
    return f"""
    <model name="passenger_{uid}">
      <static>true</static>
      <allow_auto_disable>false</allow_auto_disable>
      <pose>{x:.1f} {y:.2f} 0.30 0 0 0</pose>
      <link name="passenger_body_{uid}">
        {visual(f"leg_left_{uid}", "-0.10 0 0.36 0 0 0", "0.12 0.13 0.72", trousers)}
        {visual(f"leg_right_{uid}", "0.10 0 0.36 0 0 0", "0.12 0.13 0.72", trousers)}
        {visual(f"torso_{uid}", "0 0 1.02 0 0 0", "0.40 0.26 0.62", coat)}
        {visual(f"head_{uid}", "0 0 1.52 0 0 0", "0.24 0.24 0.28", skin)}
        {visual(f"hair_{uid}", "0 0 1.66 0 0 0", "0.25 0.25 0.08", hair)}
        {visual(f"arm_left_{uid}", "-0.26 0 1.04 0 0 0", "0.10 0.12 0.55", coat)}
        {visual(f"arm_right_{uid}", "0.26 0 1.04 0 0 0", "0.10 0.12 0.55", coat)}
      </link>
    </model>
"""


def ambient_train_model():
    body = material("0.58 0.62 0.66 1", "0.72 0.76 0.80 1")
    glass = material("0.03 0.07 0.10 1", "0.05 0.10 0.14 1")
    red = material("0.55 0.04 0.04 1", "0.75 0.06 0.06 1",
                   emissive="0.40 0.01 0.01 1")
    parts = []
    for i, x in enumerate([-50, -30, -10, 10, 30, 50]):
        parts.append(visual(f"ambient_car_{i}", f"{x} 0 0 0 0 0",
                            "19.6 2.96 3.6", body))
        parts.append(visual(f"ambient_window_{i}", f"{x} -1.505 0.55 0 0 0",
                            "17.5 0.03 0.80", glass))
    parts.append(visual("ambient_headlight", "59.9 0.85 -1.55 0 0 0",
                        "0.16 0.16 0.16", red))
    return f"""
    <model name="ambient_train">
      <static>true</static>
      <allow_auto_disable>false</allow_auto_disable>
      <pose>{TRAIN_SPAWN_X:.1f} -{TRACK_Y} 2.30 0 0 0</pose>
      <link name="ambient_train_body">
        {''.join(parts)}
      </link>
      <plugin filename="libmetro_ambient_train.so" name="metro::gazebo::AmbientTrain">
        <loop_start_x>-200.0</loop_start_x>
        <loop_end_x>200.0</loop_end_x>
        <speed_mps>9.0</speed_mps>
        <period_s>55.0</period_s>
      </plugin>
    </model>
"""



def platform_camera():
    return """
    <!-- Forward-looking platform camera (C-stage perception input). -->
    <model name="platform_camera">
      <static>true</static>
      <pose>-55 0 4.7 0 0 0</pose>
      <link name="platform_camera_link">
        <sensor name="platform_camera" type="camera">
          <always_on>true</always_on>
          <visualize>false</visualize>
          <update_rate>10</update_rate>
          <topic>/metro/camera/platform/image</topic>
          <camera>
            <horizontal_fov>1.35</horizontal_fov>
            <image><width>640</width><height>480</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>120.0</far></clip>
            <noise><type>gaussian</type><mean>0.0</mean><stddev>0.002</stddev></noise>
          </camera>
        </sensor>
      </link>
    </model>
"""


def build_world():
    frames = "".join(frame_model(i + 1, x) for i, x in enumerate(UNIT_X))
    doors = "".join(door_model(i + 1, x) for i, x in enumerate(UNIT_X))
    passengers = "".join(passenger_model(i + 1) for i in range(12))
    return f"""<?xml version="1.0" ?>
<!--
  Enriched single-line metro station (SDF 1.8).

  Layout:
    * island platform 130 x 14 m, top at z=0.30
    * train track centre y=+8.65, opposite track centre y=-8.65
    * 12 PSD units at x=-55..+55 (10 m spacing)
    * six 20 m train cars, 120 m total, alongside the platform
    * station structure, furniture, signage, tunnels, overhead line
    * 12 animated passengers, one ambient opposite train
    * DoorController, PassengerFlow and PlatformIntrusionDetector plugins
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
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>

    <scene>
      <ambient>0.58 0.60 0.64 1</ambient>
      <background>0.68 0.76 0.86 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="station_sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.92 0.90 0.84 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.35 0.25 -0.90</direction>
    </light>
    <light type="point" name="platform_fill">
      <pose>0 0 5.0 0 0 0</pose>
      <diffuse>0.55 0.58 0.64 1</diffuse>
      <specular>0.08 0.08 0.08 1</specular>
      <attenuation><range>45</range><constant>0.55</constant><linear>0.02</linear><quadratic>0.0</quadratic></attenuation>
    </light>
    <light type="point" name="platform_light_west">
      <pose>-45 0 4.8 0 0 0</pose>
      <diffuse>0.65 0.62 0.52 1</diffuse>
      <attenuation><range>40</range><constant>0.6</constant><linear>0.03</linear><quadratic>0.0</quadratic></attenuation>
    </light>
    <light type="point" name="platform_light_east">
      <pose>45 0 4.8 0 0 0</pose>
      <diffuse>0.65 0.62 0.52 1</diffuse>
      <attenuation><range>40</range><constant>0.6</constant><linear>0.03</linear><quadratic>0.0</quadratic></attenuation>
    </light>
    <light type="point" name="tunnel_light_west">
      <pose>-78 8.65 3.4 0 0 0</pose>
      <diffuse>0.30 0.38 0.50 1</diffuse>
      <attenuation><range>25</range><constant>0.5</constant><linear>0.04</linear><quadratic>0.0</quadratic></attenuation>
    </light>
    <light type="point" name="tunnel_light_east">
      <pose>78 8.65 3.4 0 0 0</pose>
      <diffuse>0.30 0.38 0.50 1</diffuse>
      <attenuation><range>25</range><constant>0.5</constant><linear>0.04</linear><quadratic>0.0</quadratic></attenuation>
    </light>

{platform_model()}
{track_model(TRACK_Y, "positive_y")}
{track_model(-TRACK_Y, "negative_y")}
{station_structure()}
{furniture()}
{signage()}
{tunnel_model(TRACK_Y, "positive_y")}
{tunnel_model(-TRACK_Y, "negative_y")}
{end_walls()}
{catenary()}
{frames}
{doors}
{passengers}
{ambient_train_model()}
{platform_camera()}

    <!-- World-level PSD actuator for all twelve units. -->
    <plugin filename="libmetro_door_controller.so" name="metro::gazebo::DoorController">
      <unit_count>{UNIT_COUNT}</unit_count>
      <unit_model_prefix>psd_door_</unit_model_prefix>
      <left_joint_prefix>psd_left_joint_</left_joint_prefix>
      <right_joint_prefix>psd_right_joint_</right_joint_prefix>
      <door_speed_mps>0.8</door_speed_mps>
      <door_travel_m>1.8</door_travel_m>
      <state_rate_hz>10.0</state_rate_hz>
      <lock_delay_s>0.3</lock_delay_s>
      <open_position_tolerance_m>0.01</open_position_tolerance_m>
    </plugin>

    <!-- Deterministic low-poly passenger flow, triggered by /metro/psd/state. -->
    <plugin filename="libmetro_passenger_flow.so" name="metro::gazebo::PassengerFlow">
      <passenger_prefix>passenger_</passenger_prefix>
      <passenger_count>12</passenger_count>
      <door_start_x>-55.0</door_start_x>
      <door_spacing_m>10.0</door_spacing_m>
      <door_y_m>6.90</door_y_m>
      <walk_speed_mps>1.25</walk_speed_mps>
      <start_delay_s>0.8</start_delay_s>
      <stagger_s>0.35</stagger_s>
    </plugin>

    <!-- Ground-truth platform-edge intrusion detector. -->
    <plugin filename="libmetro_intrusion_detector.so" name="metro::gazebo::PlatformIntrusionDetector">
      <passenger_prefix>passenger_</passenger_prefix>
      <danger_y_min_m>6.55</danger_y_min_m>
      <danger_y_max_m>7.15</danger_y_max_m>
      <publish_rate_hz>10.0</publish_rate_hz>
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
