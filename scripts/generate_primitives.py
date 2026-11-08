#!/usr/bin/env python3
"""Generate the two tiny unit meshes used by metro_description.

The SDF files scale these 1 m primitives to the real dimensions, keeping the
mesh payload small and reviewable.
"""

import math
import os

OUT_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "ws_metro",
    "src",
    "metro_description",
    "meshes",
)


def write_ascii_stl(path, triangles, name):
    lines = [f"solid {name}"]
    for a, b, c in triangles:
        ux, uy, uz = (b[i] - a[i] for i in range(3))
        vx, vy, vz = (c[i] - a[i] for i in range(3))
        nx = uy * vz - uz * vy
        ny = uz * vx - ux * vz
        nz = ux * vy - uy * vx
        length = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
        nx, ny, nz = nx / length, ny / length, nz / length
        lines.append(f"  facet normal {nx:.6f} {ny:.6f} {nz:.6f}")
        lines.append("    outer loop")
        for vertex in (a, b, c):
            lines.append(f"      vertex {vertex[0]:.6f} {vertex[1]:.6f} {vertex[2]:.6f}")
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append(f"endsolid {name}")
    with open(path, "w", encoding="ascii") as stream:
        stream.write("\n".join(lines) + "\n")


def box_triangles():
    h = 0.5
    vertices = [
        (-h, -h, -h), (h, -h, -h), (h, h, -h), (-h, h, -h),
        (-h, -h, h), (h, -h, h), (h, h, h), (-h, h, h),
    ]
    faces = [
        (0, 2, 1), (0, 3, 2),
        (4, 5, 6), (4, 6, 7),
        (0, 1, 5), (0, 5, 4),
        (1, 2, 6), (1, 6, 5),
        (2, 3, 7), (2, 7, 6),
        (3, 0, 4), (3, 4, 7),
    ]
    return [(vertices[i], vertices[j], vertices[k]) for i, j, k in faces]


def cylinder_triangles(segments=32):
    radius = 0.5
    half = 0.5
    bottom_center = (0.0, 0.0, -half)
    top_center = (0.0, 0.0, half)
    bottom = []
    top = []
    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        bottom.append((x, y, -half))
        top.append((x, y, half))

    triangles = []
    for i in range(segments):
        j = (i + 1) % segments
        triangles.append((bottom_center, (bottom[i][0], bottom[i][1], -half),
                          (bottom[j][0], bottom[j][1], -half)))
        triangles.append((top_center, (top[j][0], top[j][1], half),
                          (top[i][0], top[i][1], half)))
        triangles.append((bottom[i], bottom[j], top[j]))
        triangles.append((bottom[i], top[j], top[i]))
    return triangles


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    write_ascii_stl(os.path.join(OUT_DIR, "unit_box.stl"), box_triangles(), "unit_box")
    write_ascii_stl(os.path.join(OUT_DIR, "unit_cylinder.stl"),
                    cylinder_triangles(), "unit_cylinder")
    print(f"wrote meshes to {OUT_DIR}")


if __name__ == "__main__":
    main()
