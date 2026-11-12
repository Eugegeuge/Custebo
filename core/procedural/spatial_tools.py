"""
Advanced Spatial Manipulation & CAD Alignment Tools (`core/procedural/spatial_tools.py`).

Includes:
  1. Raycast / AABB Stack Drop-to-Surface: drops an object onto the highest supporting object
     directly underneath its XY footprint (or onto Z=0 if none).
  2. Vertex / Edge / Surface Wall Snapping: snaps a node's corner/face flush against the nearest
     neighboring node's corner/face without collision overlap.
  3. Multi-Object CAD Alignment & Even Distribution (along X, Y, or Z).
  4. 3D Euclidean Laser Measurement math (dx, dy, dz, distance, horizontal_distance, slope_deg).
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Sequence, Tuple

from core.math_physics.transforms import (
    clean_float,
    compute_local_half_extents,
    compute_world_aabb,
    euler_rpy_to_rotation_matrix,
)
from core.schema.scene_graph import SceneGraphDocument

NON_SOLID_NODE_TYPES = ("light", "ground_plane", "waypoint", "robot_spawn", "sensor", "actor")
FLAT_ROAD_ARCHETYPES = (
    "road_straight",
    "road_curved",
    "road_intersection",
    "road_crosswalk",
    "roundabout",
    "speed_bump",
)


def get_node_world_aabb(node: Dict[str, Any]) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    pose = node.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    scale = node.get("scale", [1.0, 1.0, 1.0])
    parts = node.get("parts")

    if isinstance(parts, list) and len(parts) > 0:
        px, py, pz, pr, pp, pyaw = (float(v) for v in pose[:6])
        sx, sy, sz = (float(v) for v in scale[:3])
        rot = euler_rpy_to_rotation_matrix(pr, pp, pyaw)
        min_x, min_y, min_z = 1e9, 1e9, 1e9
        max_x, max_y, max_z = -1e9, -1e9, -1e9
        for p in parts:
            lp = p.get("local_pose", p.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
            lx, ly, lz = float(lp[0]) * sx, float(lp[1]) * sy, float(lp[2]) * sz
            wx = px + rot[0][0] * lx + rot[0][1] * ly + rot[0][2] * lz
            wy = py + rot[1][0] * lx + rot[1][1] * ly + rot[1][2] * lz
            wz = pz + rot[2][0] * lx + rot[2][1] * ly + rot[2][2] * lz
            p_pose = [
                wx,
                wy,
                wz,
                pr + float(lp[3]) if len(lp) > 3 else pr,
                pp + float(lp[4]) if len(lp) > 4 else pp,
                pyaw + float(lp[5]) if len(lp) > 5 else pyaw,
            ]
            p_half = compute_local_half_extents(
                p.get("shape", "box"),
                size=p.get("size", [1.0, 1.0, 1.0]),
                radius=float(p.get("radius", 0.2)),
                length=float(p.get("length", 1.0)),
            )
            bmin, bmax = compute_world_aabb(p_pose, p_half, [sx, sy, sz])
            min_x = min(min_x, bmin[0])
            min_y = min(min_y, bmin[1])
            min_z = min(min_z, bmin[2])
            max_x = max(max_x, bmax[0])
            max_y = max(max_y, bmax[1])
            max_z = max(max_z, bmax[2])
        if min_x <= max_x:
            return ((min_x, min_y, min_z), (max_x, max_y, max_z))

    geom = node.get("geometry", {})
    half_ext = compute_local_half_extents(
        geom.get("shape", "box"),
        size=geom.get("size", [1.0, 1.0, 1.0]),
        radius=float(geom.get("radius", 0.5)),
        length=float(geom.get("length", 1.0)),
    )
    return compute_world_aabb(pose, half_ext, scale)


def compute_smart_drop_z(doc: SceneGraphDocument, target_node_id: str) -> float:
    """
    Find the highest surface (ground Z=0 or the top of any underlying object whose XY AABB
    overlaps `target_node_id`) and return the exact world Z for `target_node_id` so it rests
    flush on that surface.
    """
    target = doc.get_node(target_node_id)
    if target is None:
        return 0.0

    t_min, t_max = get_node_world_aabb(target)
    target_bottom_offset = float(target["pose"][2]) - t_min[2]

    highest_support_z = 0.0
    for other in doc.nodes:
        if other.get("id") == target_node_id:
            continue
        if other.get("type") in NON_SOLID_NODE_TYPES or not other.get("visible", True):
            continue

        o_min, o_max = get_node_world_aabb(other)
        overlap_x = (t_min[0] < o_max[0] - 1e-3) and (t_max[0] > o_min[0] + 1e-3)
        overlap_y = (t_min[1] < o_max[1] - 1e-3) and (t_max[1] > o_min[1] + 1e-3)
        if overlap_x and overlap_y:
            # Support is valid if its top is at or below the target's top
            if o_max[2] <= t_max[2] - 1e-3:
                highest_support_z = max(highest_support_z, o_max[2])

    return clean_float(highest_support_z + target_bottom_offset, 6)


def snap_node_to_nearest_neighbor(
    doc: SceneGraphDocument,
    node_id: str,
    max_snap_distance: float = 3.0,
) -> List[float]:
    """
    Snap `node_id` flush against the nearest neighboring object:
      - If `node_id` is elevated above or near the top surface of a neighbor, it can snap directly
        ONTO the neighbor's top face (Z = o_max_z + target_half_z).
      - Otherwise, it snaps flush laterally against the neighbor's +X, -X, +Y, or -Y face/corner.
    """
    target = doc.get_node(node_id)
    if target is None:
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    pose = list(target.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    t_min, t_max = get_node_world_aabb(target)
    t_hx = (t_max[0] - t_min[0]) * 0.5
    t_hy = (t_max[1] - t_min[1]) * 0.5
    t_bottom_offset = float(pose[2]) - t_min[2]

    best_dist = max_snap_distance
    best_xyz: Tuple[float, float, float] = (pose[0], pose[1], pose[2])

    for other in doc.nodes:
        if other.get("id") == node_id or other.get("type") in ("light", "ground_plane") or not other.get("visible", True):
            continue
        o_min, o_max = get_node_world_aabb(other)
        o_cx = (o_min[0] + o_max[0]) * 0.5
        o_cy = (o_min[1] + o_max[1]) * 0.5
        o_mid_z = (o_min[2] + o_max[2]) * 0.5

        # 1. Lateral flush candidates around `other` (+X, -X, +Y, -Y faces and corner-aligned)
        lateral_candidates = [
            (o_max[0] + t_hx, o_cy, pose[2]),
            (o_min[0] - t_hx, o_cy, pose[2]),
            (o_cx, o_max[1] + t_hy, pose[2]),
            (o_cx, o_min[1] - t_hy, pose[2]),
            (o_max[0] + t_hx, o_max[1] - t_hy, pose[2]),
            (o_max[0] + t_hx, o_min[1] + t_hy, pose[2]),
            (o_min[0] - t_hx, o_max[1] - t_hy, pose[2]),
            (o_min[0] - t_hx, o_min[1] + t_hy, pose[2]),
        ]

        # 2. Top-surface ("encima del objeto de abajo") snap candidates if target is above neighbor's mid-height
        #    or already overlapping its XY footprint
        overlap_xy = (t_min[0] < o_max[0] + 0.25 and t_max[0] > o_min[0] - 0.25) and (
            t_min[1] < o_max[1] + 0.25 and t_max[1] > o_min[1] - 0.25
        )
        top_z = o_max[2] + t_bottom_offset
        if t_min[2] >= o_mid_z - 0.15 or overlap_xy:
            clamped_x = min(max(pose[0], o_min[0] + min(t_hx, (o_max[0] - o_min[0]) * 0.45)), o_max[0] - min(t_hx, (o_max[0] - o_min[0]) * 0.45))
            clamped_y = min(max(pose[1], o_min[1] + min(t_hy, (o_max[1] - o_min[1]) * 0.45)), o_max[1] - min(t_hy, (o_max[1] - o_min[1]) * 0.45))
            lateral_candidates.extend(
                [
                    (o_cx, o_cy, top_z),
                    (clamped_x, clamped_y, top_z),
                ]
            )

        for cx, cy, cz in lateral_candidates:
            d = math.sqrt((cx - pose[0]) ** 2 + (cy - pose[1]) ** 2 + (cz - pose[2]) ** 2)
            if d < best_dist:
                best_dist = d
                best_xyz = (cx, cy, cz)

    pose[0] = clean_float(best_xyz[0], 6)
    pose[1] = clean_float(best_xyz[1], 6)
    pose[2] = clean_float(best_xyz[2], 6)
    return pose


def stack_node_on_target_or_nearest(
    doc: SceneGraphDocument,
    node_id: str,
    support_node_id: str | None = None,
    max_search_distance: float = 30.0,
) -> List[float]:
    """
    Place `node_id` directly ON TOP of `support_node_id` (if given) or the nearest supporting
    3D object below/nearby (`Z = support_max_z + target_bottom_offset`).
    Never mistakes flat roads/markings (`Z <= 0.12 m`) for support objects when real 3D objects exist.
    """
    target = doc.get_node(node_id)
    if target is None:
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    pose = list(target.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    t_min, t_max = get_node_world_aabb(target)
    t_hx = (t_max[0] - t_min[0]) * 0.5
    t_hy = (t_max[1] - t_min[1]) * 0.5
    t_bottom_offset = float(pose[2]) - t_min[2]

    support_node = doc.get_node(support_node_id) if support_node_id else None
    if support_node is not None and support_node.get("id") == node_id:
        support_node = None

    if support_node is None:
        # Gather all valid solid candidates (excluding target itself and non-solid markers)
        elevated_candidates: List[Tuple[Dict[str, Any], Tuple[float, float, float], Tuple[float, float, float]]] = []
        flat_candidates: List[Tuple[Dict[str, Any], Tuple[float, float, float], Tuple[float, float, float]]] = []

        for other in doc.nodes:
            if (
                other.get("id") == node_id
                or other.get("type") in NON_SOLID_NODE_TYPES
                or not other.get("visible", True)
            ):
                continue
            o_min, o_max = get_node_world_aabb(other)
            # Do not try to stack `target` onto an object that is already resting ON TOP of `target`
            if o_min[2] >= t_max[2] - 0.02 and (
                t_min[0] < o_max[0] and t_max[0] > o_min[0] and t_min[1] < o_max[1] and t_max[1] > o_min[1]
            ):
                continue
            is_flat_road = other.get("archetype") in FLAT_ROAD_ARCHETYPES or (o_max[2] - o_min[2]) <= 0.12
            if is_flat_road:
                flat_candidates.append((other, o_min, o_max))
            else:
                elevated_candidates.append((other, o_min, o_max))

        pool = elevated_candidates if elevated_candidates else flat_candidates

        # 1st priority: any 3D object in `pool` whose XY footprint already overlaps `target`
        # (prefer objects that `target` is not ALREADY resting flush on, or pick the highest one)
        overlapping: List[Tuple[Dict[str, Any], float]] = []
        for other, o_min, o_max in pool:
            if (t_min[0] < o_max[0] - 1e-3 and t_max[0] > o_min[0] + 1e-3) and (
                t_min[1] < o_max[1] - 1e-3 and t_max[1] > o_min[1] + 1e-3
            ):
                overlapping.append((other, o_max[2]))

        if overlapping:
            # Sort by top Z descending
            overlapping.sort(key=lambda item: item[1], reverse=True)
            support_node = overlapping[0][0]

        # If target is already sitting at the exact top of `support_node` (or didn't overlap anything),
        # look for the nearest 3D object in XY so pressing 'B' from beside a table/box jumps onto it!
        already_on_support = (
            support_node is not None
            and abs(t_min[2] - get_node_world_aabb(support_node)[1][2]) < 0.015
        )
        if support_node is None or already_on_support:
            best_d = max_search_distance
            nearest_other = None
            for other, o_min, o_max in pool:
                if support_node is not None and other.get("id") == support_node.get("id"):
                    continue
                # Distance from target's XY center to the closest point on `other`'s XY AABB
                clamped_x = min(max(pose[0], o_min[0]), o_max[0])
                clamped_y = min(max(pose[1], o_min[1]), o_max[1])
                d_edge = math.hypot(pose[0] - clamped_x, pose[1] - clamped_y)
                op = other.get("pose", [0, 0, 0])
                d_center = math.hypot(float(op[0]) - pose[0], float(op[1]) - pose[1])
                score_d = d_edge + 0.15 * d_center
                if score_d < best_d:
                    best_d = score_d
                    nearest_other = other
            if nearest_other is not None:
                support_node = nearest_other

    if support_node is None:
        return pose

    s_min, s_max = get_node_world_aabb(support_node)
    s_cx = (s_min[0] + s_max[0]) * 0.5
    s_cy = (s_min[1] + s_max[1]) * 0.5
    s_w = s_max[0] - s_min[0]
    s_d = s_max[1] - s_min[1]

    # If target's center is already well inside the support's XY bounds, keep its XY; otherwise center it on top
    margin_x = min(t_hx, s_w * 0.4)
    margin_y = min(t_hy, s_d * 0.4)
    inside_x = (s_min[0] + margin_x * 0.5) <= pose[0] <= (s_max[0] - margin_x * 0.5)
    inside_y = (s_min[1] + margin_y * 0.5) <= pose[1] <= (s_max[1] - margin_y * 0.5)

    new_x = pose[0] if inside_x else s_cx
    new_y = pose[1] if inside_y else s_cy
    new_z = s_max[2] + t_bottom_offset

    pose[0] = clean_float(new_x, 6)
    pose[1] = clean_float(new_y, 6)
    pose[2] = clean_float(new_z, 6)
    return pose


def align_or_distribute_nodes(
    doc: SceneGraphDocument,
    node_ids: Sequence[str],
    mode: str = "align_x_center",
    spacing: float = 1.5,
) -> Dict[str, List[float]]:
    """
    Compute new poses for a list of nodes to align or distribute them (CAD/Figma style).
    Modes:
      - `align_x`, `align_y`, `align_z_ground`
      - `distribute_x`, `distribute_y` (evenly spaced by `spacing` meters)
    Returns a dict `{node_id: new_pose}`.
    """
    valid_nodes = [doc.get_node(nid) for nid in node_ids]
    valid_nodes = [n for n in valid_nodes if n is not None and n.get("type") != "ground_plane"]
    if not valid_nodes:
        return {}

    new_poses: Dict[str, List[float]] = {}
    anchor_pose = valid_nodes[0]["pose"]

    if mode == "align_x":
        for n in valid_nodes:
            p = list(n["pose"])
            p[0] = clean_float(anchor_pose[0], 6)
            new_poses[n["id"]] = p
    elif mode == "align_y":
        for n in valid_nodes:
            p = list(n["pose"])
            p[1] = clean_float(anchor_pose[1], 6)
            new_poses[n["id"]] = p
    elif mode == "distribute_x":
        sorted_nodes = sorted(valid_nodes, key=lambda item: float(item["pose"][0]))
        x0 = float(sorted_nodes[0]["pose"][0])
        for idx, n in enumerate(sorted_nodes):
            p = list(n["pose"])
            p[0] = clean_float(x0 + idx * float(spacing), 6)
            new_poses[n["id"]] = p
    elif mode == "distribute_y":
        sorted_nodes = sorted(valid_nodes, key=lambda item: float(item["pose"][1]))
        y0 = float(sorted_nodes[0]["pose"][1])
        for idx, n in enumerate(sorted_nodes):
            p = list(n["pose"])
            p[1] = clean_float(y0 + idx * float(spacing), 6)
            new_poses[n["id"]] = p

    return new_poses


def compute_3d_ruler_measurement(
    pt_a: Sequence[float], pt_b: Sequence[float]
) -> Dict[str, float]:
    """Compute exact 3D laser ruler metrics between two ENU points (x, y, z) in meters."""
    dx = float(pt_b[0]) - float(pt_a[0])
    dy = float(pt_b[1]) - float(pt_a[1])
    dz = float(pt_b[2]) - float(pt_a[2])
    dist_xy = math.hypot(dx, dy)
    dist_3d = math.sqrt(dx * dx + dy * dy + dz * dz)
    slope_deg = math.degrees(math.atan2(abs(dz), max(dist_xy, 1e-9))) if dist_3d > 1e-9 else 0.0
    yaw_deg = math.degrees(math.atan2(dy, dx)) if dist_xy > 1e-9 else 0.0
    return {
        "dx": clean_float(dx, 4),
        "dy": clean_float(dy, 4),
        "dz": clean_float(dz, 4),
        "distance_xy": clean_float(dist_xy, 4),
        "distance_3d": clean_float(dist_3d, 4),
        "slope_deg": clean_float(slope_deg, 2),
        "yaw_deg": clean_float(yaw_deg, 2),
    }

