"""
Math and physics subpackage for Custebo.
"""

from core.math_physics.transforms import (
    clean_float,
    format_number,
    format_vec,
    euler_rpy_to_quaternion,
    quaternion_to_euler_rpy,
    euler_rpy_to_rotation_matrix,
    compute_local_half_extents,
    compute_world_aabb,
    compute_drop_to_ground_z,
)
from core.math_physics.inertia import (
    MATERIAL_PRESETS,
    SURFACE_PRESETS,
    compute_primitive_volume,
    compute_box_inertia,
    compute_cylinder_inertia,
    compute_sphere_inertia,
    apply_parallel_axis_theorem,
    compute_auto_inertial,
)

__all__ = [
    "clean_float",
    "format_number",
    "format_vec",
    "euler_rpy_to_quaternion",
    "quaternion_to_euler_rpy",
    "euler_rpy_to_rotation_matrix",
    "compute_local_half_extents",
    "compute_world_aabb",
    "compute_drop_to_ground_z",
    "MATERIAL_PRESETS",
    "SURFACE_PRESETS",
    "compute_primitive_volume",
    "compute_box_inertia",
    "compute_cylinder_inertia",
    "compute_sphere_inertia",
    "apply_parallel_axis_theorem",
    "compute_auto_inertial",
]

