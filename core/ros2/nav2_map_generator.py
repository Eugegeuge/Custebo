"""
Instant 2D Occupancy Grid Map Generator (`core/ros2/nav2_map_generator.py`).

Takes a horizontal cross-section slice of all 3D collision geometries in the Scene Graph
within a configurable height band z in [z_min, z_max] (e.g. [0.15, 0.60] m for a 2D LiDAR)
and rasterizes an exact ROS 2 / Nav2 `map.pgm` (P5 binary) + calibrated `map.yaml`.

Crucially, composite nodes (like `wall_doorway` or `wall_window`) are sliced at the
individual sub-part level in world coordinates so that doorways whose lintels are above
z_max appear as open navigable passages in the generated 2D map!
"""

from __future__ import annotations

import base64
import math
from typing import Any, Dict, List, Tuple

from core.math_physics.transforms import euler_rpy_to_rotation_matrix


def _transform_local_box_to_world_obb(
    parent_pose: List[float],
    parent_scale: List[float],
    local_pose: List[float],
    local_size: List[float],
) -> Dict[str, Any]:
    """
    Compute the world-space center, 2D XY axes, half-extents, and Z span [z0, z1]
    of a box sub-part (or root box primitive).
    """
    px, py, pz, pr, pp, pyaw = [float(v) for v in parent_pose[:6]]
    sx, sy, sz = [float(v) for v in parent_scale[:3]]
    lx, ly, lz, lr, lp, lyaw = [float(v) for v in (local_pose + [0, 0, 0, 0, 0, 0])[:6]]

    R_parent = euler_rpy_to_rotation_matrix(pr, pp, pyaw)
    # Transform local offset by scaled parent rotation
    slx, sly, slz = lx * sx, ly * sy, lz * sz
    wx = px + R_parent[0][0] * slx + R_parent[0][1] * sly + R_parent[0][2] * slz
    wy = py + R_parent[1][0] * slx + R_parent[1][1] * sly + R_parent[1][2] * slz
    wz = pz + R_parent[2][0] * slx + R_parent[2][1] * sly + R_parent[2][2] * slz

    total_yaw = pyaw + lyaw
    dx = abs(float(local_size[0]) * sx)
    dy = abs(float(local_size[1]) * sy)
    dz = abs(float(local_size[2]) * sz)

    # Compute world Z span accounting for roll/pitch if any
    R_total = euler_rpy_to_rotation_matrix(pr + lr, pp + lp, total_yaw)
    half_z_world = (
        abs(R_total[2][0]) * (dx * 0.5)
        + abs(R_total[2][1]) * (dy * 0.5)
        + abs(R_total[2][2]) * (dz * 0.5)
    )

    cos_y = math.cos(total_yaw)
    sin_y = math.sin(total_yaw)
    half_x_aabb = abs(cos_y) * (dx * 0.5) + abs(sin_y) * (dy * 0.5)
    half_y_aabb = abs(sin_y) * (dx * 0.5) + abs(cos_y) * (dy * 0.5)

    return {
        "kind": "obb",
        "cx": wx,
        "cy": wy,
        "cos_y": cos_y,
        "sin_y": sin_y,
        "hx": dx * 0.5,
        "hy": dy * 0.5,
        "z_min": wz - half_z_world,
        "z_max": wz + half_z_world,
        "min_x": wx - half_x_aabb,
        "max_x": wx + half_x_aabb,
        "min_y": wy - half_y_aabb,
        "max_y": wy + half_y_aabb,
    }


def collect_slice_primitives(
    nodes: List[Dict[str, Any]],
    z_min: float,
    z_max: float,
) -> List[Dict[str, Any]]:
    """
    Collect all 2D footprints of collision-enabled obstacles that intersect the height band [z_min, z_max].
    """
    obstacles: List[Dict[str, Any]] = []

    for node in nodes:
        if not node.get("visible", True):
            continue
        ntype = node.get("type", "primitive")
        if ntype in ("ground_plane", "light", "waypoint", "robot_spawn", "sensor", "actor"):
            continue
        col = node.get("collision", {})
        if col.get("enabled", True) is False:
            continue

        pose = node.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        scale = node.get("scale", [1.0, 1.0, 1.0])
        parts = node.get("parts")

        # 1. Multi-part composite object (e.g., doorway wall, window wall, stairs, rack, table)
        if isinstance(parts, list) and len(parts) > 0:
            for part in parts:
                p_shape = part.get("shape", "box")
                p_pose = part.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                if p_shape == "cylinder":
                    r = float(part.get("radius", 0.1))
                    l = float(part.get("length", 1.0))
                    p_size = [r * 2.0, r * 2.0, l]
                elif p_shape == "sphere":
                    r = float(part.get("radius", 0.1))
                    p_size = [r * 2.0, r * 2.0, r * 2.0]
                else:
                    p_size = part.get("size", [1.0, 1.0, 1.0])

                obb = _transform_local_box_to_world_obb(pose, scale, p_pose, p_size)
                if obb["z_max"] >= z_min and obb["z_min"] <= z_max:
                    obstacles.append(obb)
            continue

        # 2. Single primitive / mesh / include node
        geom = node.get("geometry", {})
        shape = geom.get("shape", "box")
        if shape == "cylinder":
            r = float(geom.get("radius", 0.5)) * max(float(scale[0]), float(scale[1]))
            l = float(geom.get("length", 1.0)) * float(scale[2])
            z0 = float(pose[2]) - l * 0.5
            z1 = float(pose[2]) + l * 0.5
            if z1 >= z_min and z0 <= z_max:
                obstacles.append(
                    {
                        "kind": "circle",
                        "cx": float(pose[0]),
                        "cy": float(pose[1]),
                        "radius": r,
                        "min_x": float(pose[0]) - r,
                        "max_x": float(pose[0]) + r,
                        "min_y": float(pose[1]) - r,
                        "max_y": float(pose[1]) + r,
                    }
                )
        elif shape == "sphere":
            r = float(geom.get("radius", 0.5)) * max(float(scale[0]), float(scale[1]), float(scale[2]))
            cz = float(pose[2])
            if (cz + r) >= z_min and (cz - r) <= z_max:
                # Calculate horizontal cross-section radius in [z_min, z_max]
                closest_z = max(z_min, min(cz, z_max))
                dz = abs(cz - closest_z)
                r_slice = math.sqrt(max(0.0, r * r - dz * dz))
                if r_slice > 1e-3:
                    obstacles.append(
                        {
                            "kind": "circle",
                            "cx": float(pose[0]),
                            "cy": float(pose[1]),
                            "radius": r_slice,
                            "min_x": float(pose[0]) - r_slice,
                            "max_x": float(pose[0]) + r_slice,
                            "min_y": float(pose[1]) - r_slice,
                            "max_y": float(pose[1]) + r_slice,
                        }
                    )
        else:
            bsize = geom.get("size", [1.0, 1.0, 1.0])
            if len(bsize) < 3:
                bsize = [bsize[0], bsize[1], 0.1]
            obb = _transform_local_box_to_world_obb(pose, scale, [0, 0, 0, 0, 0, 0], bsize)
            if obb["z_max"] >= z_min and obb["z_min"] <= z_max:
                obstacles.append(obb)

    return obstacles


def generate_nav2_occupancy_map(
    nodes: List[Dict[str, Any]],
    z_min: float = 0.15,
    z_max: float = 0.60,
    resolution: float = 0.05,
    padding_m: float = 2.0,
    map_filename: str = "map.pgm",
) -> Dict[str, Any]:
    """
    Generate a calibrated 2D Occupancy Grid Map (`map.pgm` + `map.yaml`) by slicing the
    scene collision geometry between `z_min` and `z_max`.

    Returns:
      {
        "width": int,
        "height": int,
        "resolution": float,
        "origin": [origin_x, origin_y, 0.0],
        "z_min": float,
        "z_max": float,
        "obstacle_count": int,
        "pgm_base64": str,
        "yaml_content": str,
        "grid_rle": List[int],  # lightweight preview array for the HTML5 canvas
      }
    """
    resolution = max(0.01, min(float(resolution), 0.5))
    z_min = float(z_min)
    z_max = max(z_min + 0.01, float(z_max))
    padding_m = max(0.5, float(padding_m))

    obstacles = collect_slice_primitives(nodes, z_min=z_min, z_max=z_max)

    if obstacles:
        min_x = min(o["min_x"] for o in obstacles) - padding_m
        max_x = max(o["max_x"] for o in obstacles) + padding_m
        min_y = min(o["min_y"] for o in obstacles) - padding_m
        max_y = max(o["max_y"] for o in obstacles) + padding_m
    else:
        min_x, max_x = -10.0, 10.0
        min_y, max_y = -10.0, 10.0

    # Snap bounds to resolution grid
    min_x = math.floor(min_x / resolution) * resolution
    min_y = math.floor(min_y / resolution) * resolution
    max_x = math.ceil(max_x / resolution) * resolution
    max_y = math.ceil(max_y / resolution) * resolution

    width = max(32, min(1200, int(round((max_x - min_x) / resolution))))
    height = max(32, min(1200, int(round((max_y - min_y) / resolution))))

    # Initialize PGM buffer: 254 = free white, 0 = occupied black
    # Note: In ROS map_server, PGM row 0 is the TOP of the image (+Y max),
    # while row (height - 1) is the BOTTOM of the image (origin_y = min_y).
    pixels = bytearray([254] * (width * height))

    for obs in obstacles:
        ix0 = max(0, int(math.floor((obs["min_x"] - min_x) / resolution)))
        ix1 = min(width - 1, int(math.ceil((obs["max_x"] - min_x) / resolution)))
        iy0 = max(0, int(math.floor((obs["min_y"] - min_y) / resolution)))
        iy1 = min(height - 1, int(math.ceil((obs["max_y"] - min_y) / resolution)))

        kind = obs["kind"]
        cx = obs["cx"]
        cy = obs["cy"]

        if kind == "circle":
            r2 = obs["radius"] * obs["radius"]
            for gy in range(iy0, iy1 + 1):
                wy = min_y + (gy + 0.5) * resolution
                dy = wy - cy
                img_row = (height - 1) - gy
                row_offset = img_row * width
                for gx in range(ix0, ix1 + 1):
                    wx = min_x + (gx + 0.5) * resolution
                    dx = wx - cx
                    if dx * dx + dy * dy <= r2:
                        pixels[row_offset + gx] = 0
        else:
            cos_y = obs["cos_y"]
            sin_y = obs["sin_y"]
            hx = obs["hx"] + resolution * 0.35
            hy = obs["hy"] + resolution * 0.35
            for gy in range(iy0, iy1 + 1):
                wy = min_y + (gy + 0.5) * resolution
                dy = wy - cy
                img_row = (height - 1) - gy
                row_offset = img_row * width
                for gx in range(ix0, ix1 + 1):
                    wx = min_x + (gx + 0.5) * resolution
                    dx = wx - cx
                    # Inverse rotate into local box frame
                    lx = cos_y * dx + sin_y * dy
                    ly = -sin_y * dx + cos_y * dy
                    if abs(lx) <= hx and abs(ly) <= hy:
                        pixels[row_offset + gx] = 0

    header = f"P5\n# Generated by Custebo 3D World Editor (z_slice=[{z_min:.2f}, {z_max:.2f}]m)\n{width} {height}\n255\n".encode(
        "ascii"
    )
    pgm_bytes = header + bytes(pixels)

    yaml_lines = [
        f"image: {map_filename}",
        "mode: trinary",
        f"resolution: {resolution:.4f}",
        f"origin: [{min_x:.4f}, {min_y:.4f}, 0.0]",
        "negate: 0",
        "occupied_thresh: 0.65",
        "free_thresh: 0.196",
        "",
    ]
    yaml_content = "\n".join(yaml_lines)

    return {
        "width": width,
        "height": height,
        "resolution": round(resolution, 4),
        "origin": [round(min_x, 4), round(min_y, 4), 0.0],
        "z_min": round(z_min, 3),
        "z_max": round(z_max, 3),
        "obstacle_count": len(obstacles),
        "pgm_bytes": pgm_bytes,
        "pgm_base64": base64.b64encode(pgm_bytes).decode("ascii"),
        "yaml_content": yaml_content,
    }
