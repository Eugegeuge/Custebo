"""
Scene Graph Schema Package.
"""

from core.schema.scene_graph import (
    SCHEMA_VERSION,
    SceneGraphDocument,
    generate_node_id,
    normalize_node_in_place,
    recalculate_node_inertial,
    create_primitive_node,
    create_ground_plane_node,
    create_light_node,
)
from core.schema.presets_library import (
    ARCHETYPE_CATALOG,
    create_archetype_node,
    rebuild_archetype_node,
)

__all__ = [
    "SCHEMA_VERSION",
    "SceneGraphDocument",
    "generate_node_id",
    "normalize_node_in_place",
    "recalculate_node_inertial",
    "create_primitive_node",
    "create_ground_plane_node",
    "create_light_node",
    "ARCHETYPE_CATALOG",
    "create_archetype_node",
    "rebuild_archetype_node",
]
