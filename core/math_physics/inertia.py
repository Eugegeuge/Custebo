"""
Automatic Physics, Volume, Density, and Inertia Tensor Calculator (ENU Z-Up).
Units:
  - Mass: kg
  - Density: kg/m^3
  - Dimensions: m
  - Inertia tensor: kg * m^2
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Sequence, Tuple
from core.math_physics.transforms import clean_float


MATERIAL_PRESETS: Dict[str, Dict[str, float | str]] = {
    "wood": {"label": "Madera (Pino/Roble)", "density": 650.0, "mu1": 0.5, "mu2": 0.5, "restitution": 0.2},
    "plastic": {"label": "Plástico (ABS/PVC)", "density": 1150.0, "mu1": 0.4, "mu2": 0.4, "restitution": 0.3},
    "concrete": {"label": "Hormigón", "density": 2400.0, "mu1": 0.8, "mu2": 0.8, "restitution": 0.05},
    "aluminum": {"label": "Aluminio 6061", "density": 2700.0, "mu1": 0.45, "mu2": 0.45, "restitution": 0.15},
    "steel": {"label": "Acero Estructural", "density": 7850.0, "mu1": 0.6, "mu2": 0.6, "restitution": 0.1},
    "rubber": {"label": "Goma / Neumático", "density": 1100.0, "mu1": 1.0, "mu2": 1.0, "restitution": 0.6},
    "ice": {"label": "Hielo", "density": 917.0, "mu1": 0.03, "mu2": 0.03, "restitution": 0.05},
}

SURFACE_PRESETS: Dict[str, Dict[str, float]] = {
    "warehouse_floor": {"mu1": 0.7, "mu2": 0.7, "restitution": 0.05, "kp": 1e7, "kd": 1.0},
    "asphalt": {"mu1": 0.9, "mu2": 0.9, "restitution": 0.05, "kp": 1e7, "kd": 1.0},
    "ice": {"mu1": 0.03, "mu2": 0.03, "restitution": 0.05, "kp": 1e7, "kd": 1.0},
    "dirt": {"mu1": 0.65, "mu2": 0.65, "restitution": 0.02, "kp": 5e6, "kd": 10.0},
    "rubber": {"mu1": 1.0, "mu2": 1.0, "restitution": 0.5, "kp": 1e6, "kd": 10.0},
}


def compute_primitive_volume(
    shape: str,
    size: Sequence[float] = (1.0, 1.0, 1.0),
    radius: float = 0.5,
    length: float = 1.0,
    scale: Sequence[float] = (1.0, 1.0, 1.0),
) -> float:
    """Compute volume in m^3 of a scaled primitive."""
    sx, sy, sz = (abs(float(scale[0])), abs(float(scale[1])), abs(float(scale[2])))
    if shape == "box":
        dx = abs(float(size[0])) * sx
        dy = abs(float(size[1])) * sy
        dz = abs(float(size[2])) * sz
        return max(dx * dy * dz, 1e-9)
    if shape == "cylinder":
        rx = abs(float(radius)) * sx
        ry = abs(float(radius)) * sy
        h = abs(float(length)) * sz
        return max(math.pi * rx * ry * h, 1e-9)
    if shape == "sphere":
        rx = abs(float(radius)) * sx
        ry = abs(float(radius)) * sy
        rz = abs(float(radius)) * sz
        return max((4.0 / 3.0) * math.pi * rx * ry * rz, 1e-9)
    if shape == "capsule":
        r = abs(float(radius)) * 0.5 * (sx + sy)
        h = abs(float(length)) * sz
        cyl_vol = math.pi * r * r * h
        sph_vol = (4.0 / 3.0) * math.pi * (r ** 3)
        return max(cyl_vol + sph_vol, 1e-9)
    return 1.0


def compute_box_inertia(mass: float, dx: float, dy: float, dz: float) -> Dict[str, float]:
    """Compute inertia tensor of a solid box of dimensions (dx, dy, dz) in m and mass in kg."""
    if mass <= 0 or dx <= 0 or dy <= 0 or dz <= 0:
        raise ValueError("Mass and box dimensions (dx, dy, dz) must be > 0.")
    c = mass / 12.0
    return {
        "ixx": clean_float(c * (dy * dy + dz * dz)),
        "iyy": clean_float(c * (dx * dx + dz * dz)),
        "izz": clean_float(c * (dx * dx + dy * dy)),
        "ixy": 0.0,
        "ixz": 0.0,
        "iyz": 0.0,
    }


def compute_cylinder_inertia(mass: float, radius: float, length: float) -> Dict[str, float]:
    """Compute inertia tensor of a Z-aligned solid cylinder of radius and length in m."""
    if mass <= 0 or radius <= 0 or length <= 0:
        raise ValueError("Mass, radius, and length must be > 0.")
    ixx_iyy = (mass / 12.0) * (3.0 * radius * radius + length * length)
    izz = 0.5 * mass * radius * radius
    return {
        "ixx": clean_float(ixx_iyy),
        "iyy": clean_float(ixx_iyy),
        "izz": clean_float(izz),
        "ixy": 0.0,
        "ixz": 0.0,
        "iyz": 0.0,
    }


def compute_sphere_inertia(mass: float, radius: float) -> Dict[str, float]:
    """Compute inertia tensor of a solid sphere of radius (m) and mass (kg)."""
    if mass <= 0 or radius <= 0:
        raise ValueError("Mass and radius must be > 0.")
    val = clean_float(0.4 * mass * radius * radius)
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
    """Shift an inertia tensor from CoM by offset (dx, dy, dz) using Steiner's theorem."""
    dx, dy, dz = (float(com_offset[0]), float(com_offset[1]), float(com_offset[2]))
    d2 = dx * dx + dy * dy + dz * dz
    return {
        "ixx": clean_float(inertia_cm["ixx"] + mass * (d2 - dx * dx)),
        "iyy": clean_float(inertia_cm["iyy"] + mass * (d2 - dy * dy)),
        "izz": clean_float(inertia_cm["izz"] + mass * (d2 - dz * dz)),
        "ixy": clean_float(inertia_cm.get("ixy", 0.0) - mass * dx * dy),
        "ixz": clean_float(inertia_cm.get("ixz", 0.0) - mass * dx * dz),
        "iyz": clean_float(inertia_cm.get("iyz", 0.0) - mass * dy * dz),
    }


def compute_auto_inertial(
    shape: str,
    size: Sequence[float] = (1.0, 1.0, 1.0),
    radius: float = 0.5,
    length: float = 1.0,
    scale: Sequence[float] = (1.0, 1.0, 1.0),
    mass: Optional[float] = None,
    material_preset: Optional[str] = None,
    com_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Dict[str, float]:
    """
    Calculate mass and positive-definite inertia tensor for any scaled primitive or bounding box.
    """
    sx, sy, sz = (max(abs(float(scale[0])), 1e-4), max(abs(float(scale[1])), 1e-4), max(abs(float(scale[2])), 1e-4))
    vol = compute_primitive_volume(shape, size=size, radius=radius, length=length, scale=(sx, sy, sz))

    if mass is None or mass <= 0.0:
        preset_key = (material_preset or "wood").lower()
        density = float(MATERIAL_PRESETS.get(preset_key, MATERIAL_PRESETS["wood"])["density"])
        computed_mass = max(clean_float(vol * density, decimals=4), 0.001)
    else:
        computed_mass = max(clean_float(float(mass), decimals=6), 0.001)

    if shape == "cylinder":
        eff_r = max(abs(float(radius)) * 0.5 * (sx + sy), 1e-3)
        eff_l = max(abs(float(length)) * sz, 1e-3)
        tensor = compute_cylinder_inertia(computed_mass, eff_r, eff_l)
    elif shape == "sphere":
        eff_r = max(abs(float(radius)) * ((sx + sy + sz) / 3.0), 1e-3)
        tensor = compute_sphere_inertia(computed_mass, eff_r)
    else:
        dx = max(abs(float(size[0])) * sx, 1e-3)
        dy = max(abs(float(size[1])) * sy, 1e-3)
        dz = max(abs(float(size[2])) * sz, 1e-3)
        tensor = compute_box_inertia(computed_mass, dx, dy, dz)

    if any(abs(c) > 1e-9 for c in com_offset):
        tensor = apply_parallel_axis_theorem(tensor, computed_mass, com_offset)

    # Guarantee minimum positive diagonal values for numerical stability in Gazebo DART
    for k in ("ixx", "iyy", "izz"):
        if tensor[k] < 1e-6:
            tensor[k] = 1e-6

    return {
        "mass": computed_mass,
        **tensor,
    }

