"""
Pure 3D Math, ENU Z-Up Transformations, Quaternions, and Bounding Box Utilities.
All distances in meters (m) and angles in radians (rad).
"""

from __future__ import annotations

import math
from typing import Sequence, Tuple

Pose6D = Tuple[float, float, float, float, float, float]
Vec3 = Tuple[float, float, float]
QuatXYZW = Tuple[float, float, float, float]
Mat3x3 = Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]


def clean_float(val: float, decimals: int = 6, eps: float = 1e-11) -> float:
    """Eliminate -0.0 and round cleanly for deterministic serialization."""
    if not math.isfinite(val) or abs(val) < eps:
        return 0.0
    rounded = round(float(val), decimals)
    return 0.0 if abs(rounded) < eps else rounded


def format_number(val: float, decimals: int = 6) -> str:
    """Format a float deterministically without trailing zeros or '-0'."""
    cleaned = clean_float(val, decimals=decimals)
    if cleaned == 0.0:
        return "0"
    text = f"{cleaned:.{decimals}f}".rstrip("0").rstrip(".")
    return "0" if text in ("", "-0") else text


def format_vec(values: Sequence[float], decimals: int = 6) -> str:
    """Format a vector/pose sequence as space-separated deterministic numbers."""
    return " ".join(format_number(float(v), decimals=decimals) for v in values)


def euler_rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> QuatXYZW:
    """
    Convert ENU Roll (X), Pitch (Y), Yaw (Z) in radians (intrinsic ZYX / extrinsic XYZ)
    to unit quaternion (qx, qy, qz, qw).
    """
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return (qx, qy, qz, qw)


def quaternion_to_euler_rpy(qx: float, qy: float, qz: float, qw: float) -> Vec3:
    """
    Convert unit quaternion (qx, qy, qz, qw) to ENU (roll, pitch, yaw) in radians.
    """
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        raise ValueError("Zero-norm quaternion is invalid.")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0 - 1e-12:
        pitch = math.copysign(math.pi / 2.0, sinp)
        roll = 0.0
        yaw = math.atan2(-2.0 * (qx * qy - qw * qz), 1.0 - 2.0 * (qx * qx + qz * qz))
    else:
        pitch = math.asin(sinp)
        roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    return (roll, pitch, yaw)


def euler_rpy_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> Mat3x3:
    """Return 3x3 rotation matrix R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def compute_local_half_extents(
    shape: str,
    size: Sequence[float] = (1.0, 1.0, 1.0),
    radius: float = 0.5,
    length: float = 1.0,
) -> Vec3:
    """Return unscaled local half-extents (hx, hy, hz) for a primitive shape."""
    if shape == "box":
        return (float(size[0]) * 0.5, float(size[1]) * 0.5, float(size[2]) * 0.5)
    if shape == "cylinder":
        r = float(radius)
        return (r, r, float(length) * 0.5)
    if shape == "sphere":
        r = float(radius)
        return (r, r, r)
    if shape == "capsule":
        r = float(radius)
        return (r, r, float(length) * 0.5 + r)
    if shape == "plane":
        sx = float(size[0]) if len(size) >= 1 else 10.0
        sy = float(size[1]) if len(size) >= 2 else 10.0
        return (sx * 0.5, sy * 0.5, 0.001)
    return (0.5, 0.5, 0.5)


def compute_world_aabb(
    pose: Sequence[float],
    local_half_extents: Sequence[float],
    scale: Sequence[float] = (1.0, 1.0, 1.0),
) -> Tuple[Vec3, Vec3]:
    """
    Compute exact world-space AABB (min_xyz, max_xyz) for an object centered at `pose`
    with given `local_half_extents` and `scale`.
    """
    px, py, pz, r, p, y = (float(v) for v in pose[:6])
    hx = abs(float(local_half_extents[0]) * float(scale[0]))
    hy = abs(float(local_half_extents[1]) * float(scale[1]))
    hz = abs(float(local_half_extents[2]) * float(scale[2]))

    rot = euler_rpy_to_rotation_matrix(r, p, y)
    ex = abs(rot[0][0]) * hx + abs(rot[0][1]) * hy + abs(rot[0][2]) * hz
    ey = abs(rot[1][0]) * hx + abs(rot[1][1]) * hy + abs(rot[1][2]) * hz
    ez = abs(rot[2][0]) * hx + abs(rot[2][1]) * hy + abs(rot[2][2]) * hz

    return ((px - ex, py - ey, pz - ez), (px + ex, py + ey, pz + ez))


def compute_drop_to_ground_z(
    pose: Sequence[float],
    local_half_extents: Sequence[float],
    ground_z: float = 0.0,
    scale: Sequence[float] = (1.0, 1.0, 1.0),
) -> float:
    """
    Calculate the world Z coordinate so the object's lowest point rests exactly on `ground_z`.
    """
    aabb_min, _ = compute_world_aabb(pose, local_half_extents, scale)
    delta_z = float(ground_z) - aabb_min[2]
    return clean_float(float(pose[2]) + delta_z)

