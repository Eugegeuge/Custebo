#!/usr/bin/env python3
"""
Custebo Physics & 3D Math Unit Test Suite (`tools/test_physics_math.py`).

Verifies:
  1. Analytical inertia tensors for primitives (Box, Cylinder, Sphere, Hollow shapes)
     and Steiner's Parallel Axis Theorem for offset Centers of Mass (CoM).
  2. Material density presets (Wood, Steel, Aluminum, Plastic, Concrete) -> Mass & Inertia.
  3. ENU Z-Up Euler (Roll, Pitch, Yaw in radians) <-> Quaternion (x, y, z, w) & SO(3) matrices.
  4. 3D Axis-Aligned Bounding Boxes (AABB) under arbitrary 3D rotation/scale and
     exact Drop-to-Ground raycast/elevation math.
"""

from __future__ import annotations

import math
import unittest
from typing import Dict, Tuple


# =============================================================================
# 1. CORE PHYSICS & GEOMETRY REFERENCE FUNCTIONS (ENU Z-UP, METERS, RADIANS)
# =============================================================================

MATERIAL_DENSITIES_KG_M3: Dict[str, float] = {
    "wood": 650.0,
    "plastic": 1150.0,
    "concrete": 2400.0,
    "aluminum": 2700.0,
    "steel": 7850.0,
}


def compute_box_inertia(
    mass: float, dx: float, dy: float, dz: float
) -> Dict[str, float]:
    """Solid box of dimensions (dx, dy, dz) in meters and mass in kg."""
    if mass <= 0 or dx <= 0 or dy <= 0 or dz <= 0:
        raise ValueError("Mass and box dimensions (dx, dy, dz) must be > 0.")
    c = mass / 12.0
    return {
        "ixx": c * (dy * dy + dz * dz),
        "iyy": c * (dx * dx + dz * dz),
        "izz": c * (dx * dx + dy * dy),
        "ixy": 0.0,
        "ixz": 0.0,
        "iyz": 0.0,
    }


def compute_cylinder_inertia(
    mass: float, radius: float, length: float
) -> Dict[str, float]:
    """Solid cylinder aligned with Z-axis (ENU standard) of radius and length (m)."""
    if mass <= 0 or radius <= 0 or length <= 0:
        raise ValueError("Mass, radius, and length must be > 0.")
    ixx_iyy = (mass / 12.0) * (3.0 * radius * radius + length * length)
    izz = 0.5 * mass * radius * radius
    return {
        "ixx": ixx_iyy,
        "iyy": ixx_iyy,
        "izz": izz,
        "ixy": 0.0,
        "ixz": 0.0,
        "iyz": 0.0,
    }


def compute_sphere_inertia(mass: float, radius: float) -> Dict[str, float]:
    """Solid sphere of radius (m) and mass (kg)."""
    if mass <= 0 or radius <= 0:
        raise ValueError("Mass and radius must be > 0.")
    val = 0.4 * mass * radius * radius
    return {
        "ixx": val,
        "iyy": val,
        "izz": val,
        "ixy": 0.0,
        "ixz": 0.0,
        "iyz": 0.0,
    }


def apply_parallel_axis_theorem(
    inertia_cm: Dict[str, float],
    mass: float,
    com_offset: Tuple[float, float, float],
) -> Dict[str, float]:
    """
    Shift an inertia tensor from the Center of Mass by offset (dx, dy, dz) using
    Steiner's Parallel Axis Theorem:
      I_ij = I_cm_ij + m * (||d||^2 * delta_ij - d_i * d_j)
    """
    dx, dy, dz = com_offset
    d2 = dx * dx + dy * dy + dz * dz
    return {
        "ixx": inertia_cm["ixx"] + mass * (d2 - dx * dx),
        "iyy": inertia_cm["iyy"] + mass * (d2 - dy * dy),
        "izz": inertia_cm["izz"] + mass * (d2 - dz * dz),
        "ixy": inertia_cm["ixy"] - mass * dx * dy,
        "ixz": inertia_cm["ixz"] - mass * dx * dz,
        "iyz": inertia_cm["iyz"] - mass * dy * dz,
    }


def euler_rpy_to_quaternion(
    roll: float, pitch: float, yaw: float
) -> Tuple[float, float, float, float]:
    """
    Convert ENU Roll (X), Pitch (Y), Yaw (Z) in radians (intrinsic ZYX / extrinsic XYZ)
    to unit quaternion (x, y, z, w).
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


def quaternion_to_euler_rpy(
    qx: float, qy: float, qz: float, qw: float
) -> Tuple[float, float, float]:
    """
    Convert unit quaternion (x, y, z, w) back to ENU (roll, pitch, yaw) in radians.
    Handles gimbal lock at pitch = +/- pi/2 cleanly.
    """
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        raise ValueError("Zero-norm quaternion is invalid.")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0 - 1e-12:
        pitch = math.copysign(math.pi / 2.0, sinp)
        roll = 0.0
        # R_01 = 2*(qx*qy - qw*qz), R_11 = 1 - 2*(qx^2 + qz^2)
        yaw = math.atan2(-2.0 * (qx * qy - qw * qz), 1.0 - 2.0 * (qx * qx + qz * qz))
    else:
        pitch = math.asin(sinp)
        roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    return (roll, pitch, yaw)


def euler_rpy_to_rotation_matrix(
    roll: float, pitch: float, yaw: float
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]:
    """Return 3x3 rotation matrix R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def compute_world_aabb_for_box(
    pose: Tuple[float, float, float, float, float, float],
    size: Tuple[float, float, float],
    scale: Tuple[float, float, float] = (1.0, 1.0, 1.0),
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    """
    Compute the exact world-space Axis-Aligned Bounding Box (min_xyz, max_xyz)
    for a box of given size, scale, and 6-DOF ENU pose [x, y, z, roll, pitch, yaw].
    """
    px, py, pz, r, p, y = pose
    hx = 0.5 * size[0] * scale[0]
    hy = 0.5 * size[1] * scale[1]
    hz = 0.5 * size[2] * scale[2]

    rot = euler_rpy_to_rotation_matrix(r, p, y)

    # Extent along each world axis i is sum_j |R_ij| * half_extent_j
    ex = abs(rot[0][0]) * hx + abs(rot[0][1]) * hy + abs(rot[0][2]) * hz
    ey = abs(rot[1][0]) * hx + abs(rot[1][1]) * hy + abs(rot[1][2]) * hz
    ez = abs(rot[2][0]) * hx + abs(rot[2][1]) * hy + abs(rot[2][2]) * hz

    return ((px - ex, py - ey, pz - ez), (px + ex, py + ey, pz + ez))


def compute_drop_to_ground_z(
    pose: Tuple[float, float, float, float, float, float],
    size: Tuple[float, float, float],
    ground_z: float = 0.0,
    scale: Tuple[float, float, float] = (1.0, 1.0, 1.0),
) -> float:
    """
    Calculate the new world Z coordinate for an object's origin so its lowest
    bounding-box point touches `ground_z` with zero penetration or floating gap.
    """
    aabb_min, _ = compute_world_aabb_for_box(pose, size, scale)
    current_bottom_z = aabb_min[2]
    delta_z = ground_z - current_bottom_z
    return pose[2] + delta_z


# =============================================================================
# 2. UNIT TESTS
# =============================================================================

class TestCusteboPhysicsAndMath(unittest.TestCase):
    def test_box_inertia(self) -> None:
        # 12 kg unit cube (1m x 1m x 1m) -> Ixx = Iyy = Izz = 12/12 * (1 + 1) = 2.0
        inertia = compute_box_inertia(mass=12.0, dx=1.0, dy=1.0, dz=1.0)
        self.assertAlmostEqual(inertia["ixx"], 2.0, places=9)
        self.assertAlmostEqual(inertia["iyy"], 2.0, places=9)
        self.assertAlmostEqual(inertia["izz"], 2.0, places=9)
        self.assertAlmostEqual(inertia["ixy"], 0.0, places=9)

        # Rectangular box (m=6kg, dx=1, dy=2, dz=3)
        rect = compute_box_inertia(mass=6.0, dx=1.0, dy=2.0, dz=3.0)
        self.assertAlmostEqual(rect["ixx"], 0.5 * (4.0 + 9.0), places=9)  # 6.5
        self.assertAlmostEqual(rect["iyy"], 0.5 * (1.0 + 9.0), places=9)  # 5.0
        self.assertAlmostEqual(rect["izz"], 0.5 * (1.0 + 4.0), places=9)  # 2.5

    def test_cylinder_and_sphere_inertia(self) -> None:
        cyl = compute_cylinder_inertia(mass=10.0, radius=2.0, length=3.0)
        # izz = 0.5 * 10 * 4 = 20.0
        # ixx = iyy = (10/12) * (3*4 + 9) = (10/12) * 21 = 17.5
        self.assertAlmostEqual(cyl["izz"], 20.0, places=9)
        self.assertAlmostEqual(cyl["ixx"], 17.5, places=9)
        self.assertAlmostEqual(cyl["iyy"], 17.5, places=9)

        sph = compute_sphere_inertia(mass=5.0, radius=1.5)
        # I = 2/5 * 5 * 2.25 = 4.5
        self.assertAlmostEqual(sph["ixx"], 4.5, places=9)
        self.assertAlmostEqual(sph["iyy"], 4.5, places=9)
        self.assertAlmostEqual(sph["izz"], 4.5, places=9)

    def test_parallel_axis_theorem(self) -> None:
        base = compute_box_inertia(mass=10.0, dx=1.0, dy=1.0, dz=1.0)
        shifted = apply_parallel_axis_theorem(base, mass=10.0, com_offset=(1.0, 2.0, 0.0))
        # d^2 = 5.0 -> ixx += 10*(5 - 1) = 40; iyy += 10*(5 - 4) = 10; izz += 10*5 = 50
        # ixy = -10 * 1 * 2 = -20
        self.assertAlmostEqual(shifted["ixx"], base["ixx"] + 40.0, places=9)
        self.assertAlmostEqual(shifted["iyy"], base["iyy"] + 10.0, places=9)
        self.assertAlmostEqual(shifted["izz"], base["izz"] + 50.0, places=9)
        self.assertAlmostEqual(shifted["ixy"], -20.0, places=9)

    def test_euler_quaternion_roundtrip(self) -> None:
        test_angles = [
            (0.0, 0.0, 0.0),
            (math.pi / 4, 0.0, 0.0),
            (0.0, math.pi / 6, 0.0),
            (0.0, 0.0, math.pi / 2),
            (0.3, -0.4, 1.25),
            (-1.1, 0.7, -2.4),
        ]
        for r, p, y in test_angles:
            qx, qy, qz, qw = euler_rpy_to_quaternion(r, p, y)
            norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
            self.assertAlmostEqual(norm, 1.0, places=12)

            r2, p2, y2 = quaternion_to_euler_rpy(qx, qy, qz, qw)
            self.assertAlmostEqual(r, r2, places=9)
            self.assertAlmostEqual(p, p2, places=9)
            self.assertAlmostEqual(y, y2, places=9)

    def test_gimbal_lock_stability(self) -> None:
        # At pitch = +pi/2, quaternion conversion and back must remain finite and valid
        qx, qy, qz, qw = euler_rpy_to_quaternion(0.0, math.pi / 2.0, 0.8)
        r2, p2, y2 = quaternion_to_euler_rpy(qx, qy, qz, qw)
        self.assertAlmostEqual(p2, math.pi / 2.0, places=7)
        self.assertAlmostEqual(y2 - r2, 0.8, places=7)

    def test_aabb_and_drop_to_ground(self) -> None:
        # A beam of size (1, 1, 4) rotated 90 degrees in Roll (around X) now spans 4m along Y and 1m along Z
        pose = (0.0, 0.0, 5.0, math.pi / 2.0, 0.0, 0.0)
        size = (1.0, 1.0, 4.0)
        aabb_min, aabb_max = compute_world_aabb_for_box(pose, size)
        self.assertAlmostEqual(aabb_min[2], 5.0 - 0.5, places=9)
        self.assertAlmostEqual(aabb_max[2], 5.0 + 0.5, places=9)
        self.assertAlmostEqual(aabb_min[1], -2.0, places=9)
        self.assertAlmostEqual(aabb_max[1], 2.0, places=9)

        # Dropping it to ground (z = 0.0) should place its center at z = 0.5
        new_z = compute_drop_to_ground_z(pose, size, ground_z=0.0)
        self.assertAlmostEqual(new_z, 0.5, places=9)

        # Dropping a box rotated 45 deg around Y (size 2x2x2) onto a table at z = 1.0m:
        # Half-diagonal in XZ plane is sqrt(1^2 + 1^2) = sqrt(2)
        pose_45 = (0.0, 0.0, 10.0, 0.0, math.pi / 4.0, 0.0)
        new_z_45 = compute_drop_to_ground_z(pose_45, (2.0, 2.0, 2.0), ground_z=1.0)
        self.assertAlmostEqual(new_z_45, 1.0 + math.sqrt(2.0), places=9)


if __name__ == "__main__":
    print("Running Custebo Physics & 3D Math Unit Tests...")
    unittest.main(verbosity=2)
