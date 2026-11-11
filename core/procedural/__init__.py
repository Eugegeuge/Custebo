"""
Procedural & CAD Tools Subpackage for Custebo (Phase 3).
"""

from core.procedural.spatial_tools import (
    get_node_world_aabb,
    compute_smart_drop_z,
    snap_node_to_nearest_neighbor,
    align_or_distribute_nodes,
    compute_3d_ruler_measurement,
)
from core.procedural.floorplan_tracer import (
    extrude_polyline_to_walls,
    parse_pgm_bytes,
    reconstruct_3d_walls_from_pgm,
)
from core.procedural.cloners import (
    generate_grid_array,
    evaluate_cubic_bezier,
    generate_spline_clones,
    generate_scatter_brush,
)
from core.procedural.terrain_generator import create_parametric_terrain_node

__all__ = [
    "get_node_world_aabb",
    "compute_smart_drop_z",
    "snap_node_to_nearest_neighbor",
    "align_or_distribute_nodes",
    "compute_3d_ruler_measurement",
    "extrude_polyline_to_walls",
    "parse_pgm_bytes",
    "reconstruct_3d_walls_from_pgm",
    "generate_grid_array",
    "evaluate_cubic_bezier",
    "generate_spline_clones",
    "generate_scatter_brush",
    "create_parametric_terrain_node",
]
