"""
Concrete Scene Graph Commands (`core/commands/node_commands.py`).
All editor tools operate through these reversible commands.
"""

from __future__ import annotations

import copy
from typing import Any, Dict, List, Optional
from core.commands.base import Command
from core.math_physics.transforms import (
    clean_float,
    compute_drop_to_ground_z,
    compute_local_half_extents,
    euler_rpy_to_rotation_matrix,
)
from core.procedural.spatial_tools import compute_smart_drop_z, get_node_world_aabb
from core.schema.presets_library import rebuild_archetype_node
from core.schema.scene_graph import (
    SceneGraphDocument,
    create_primitive_node,
    generate_node_id,
    normalize_node_in_place,
    recalculate_node_inertial,
)


def _deep_merge_dict(target: Dict[str, Any], patch: Dict[str, Any]) -> None:
    for key, val in patch.items():
        if isinstance(val, dict) and isinstance(target.get(key), dict):
            _deep_merge_dict(target[key], val)
        else:
            target[key] = copy.deepcopy(val)


class AddNodeCommand(Command):
    """Add a new node (primitive, light, mesh, etc.) to the Scene Graph."""

    def __init__(self, node_data: Dict[str, Any]) -> None:
        self.node_data = normalize_node_in_place(copy.deepcopy(node_data))
        super().__init__(label=f"Add '{self.node_data['name']}'")

    def execute(self, doc: SceneGraphDocument) -> None:
        self.node_data["name"] = doc.ensure_unique_name(
            self.node_data["name"], exclude_id=self.node_data["id"]
        )
        doc.nodes.append(copy.deepcopy(self.node_data))

    def undo(self, doc: SceneGraphDocument) -> None:
        doc.nodes = [n for n in doc.nodes if n.get("id") != self.node_data["id"]]


class DeleteNodesCommand(Command):
    """Delete one or more nodes by ID, preserving their index order for exact Undo."""

    def __init__(self, node_ids: List[str]) -> None:
        self.node_ids = list(dict.fromkeys(node_ids))
        self.removed_entries: List[tuple[int, Dict[str, Any]]] = []
        super().__init__(label=f"Delete {len(self.node_ids)} object(s)")

    def execute(self, doc: SceneGraphDocument) -> None:
        self.removed_entries.clear()
        id_set = set(self.node_ids)
        remaining: List[Dict[str, Any]] = []
        for idx, node in enumerate(doc.nodes):
            if node.get("id") in id_set:
                self.removed_entries.append((idx, copy.deepcopy(node)))
            else:
                remaining.append(node)
        doc.nodes = remaining

    def undo(self, doc: SceneGraphDocument) -> None:
        for idx, saved_node in sorted(self.removed_entries, key=lambda x: x[0]):
            insert_at = min(idx, len(doc.nodes))
            doc.nodes.insert(insert_at, copy.deepcopy(saved_node))


class SwapNodesOrderCommand(Command):
    """Swap the order of two nodes in `doc.nodes` (used for Nav2 waypoint route ordering)."""

    def __init__(self, node_id_a: str, node_id_b: str, label: str = "Reorder Waypoints") -> None:
        self.node_id_a = node_id_a
        self.node_id_b = node_id_b
        super().__init__(label=label)

    def _swap(self, doc: SceneGraphDocument) -> None:
        idx_a = next((i for i, n in enumerate(doc.nodes) if n.get("id") == self.node_id_a), -1)
        idx_b = next((i for i, n in enumerate(doc.nodes) if n.get("id") == self.node_id_b), -1)
        if idx_a != -1 and idx_b != -1 and idx_a != idx_b:
            doc.nodes[idx_a], doc.nodes[idx_b] = doc.nodes[idx_b], doc.nodes[idx_a]

    def execute(self, doc: SceneGraphDocument) -> None:
        self._swap(doc)

    def undo(self, doc: SceneGraphDocument) -> None:
        self._swap(doc)



class DuplicateNodeCommand(Command):
    """Duplicate an existing node (`Ctrl+D`) with a slight offset (+0.25m X, +0.25m Y)."""

    def __init__(self, source_node_id: str, offset_xy: tuple[float, float] = (0.25, 0.25)) -> None:
        self.source_node_id = source_node_id
        self.offset_xy = offset_xy
        self.created_node: Optional[Dict[str, Any]] = None
        super().__init__(label="Duplicate Object")

    def execute(self, doc: SceneGraphDocument) -> None:
        if self.created_node is None:
            src = doc.get_node(self.source_node_id)
            if src is None:
                return
            dup = copy.deepcopy(src)
            dup["id"] = generate_node_id(dup.get("type", "node"))
            dup["name"] = doc.ensure_unique_name(f"{src.get('name', 'object')}_copy")
            dup["locked"] = False
            pose = list(dup.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
            pose[0] = round(float(pose[0]) + self.offset_xy[0], 6)
            pose[1] = round(float(pose[1]) + self.offset_xy[1], 6)
            dup["pose"] = pose
            self.created_node = normalize_node_in_place(dup)
            self.label = f"Duplicate '{src.get('name')}' -> '{self.created_node['name']}'"
        doc.nodes.append(copy.deepcopy(self.created_node))

    def undo(self, doc: SceneGraphDocument) -> None:
        if self.created_node:
            cid = self.created_node["id"]
            doc.nodes = [n for n in doc.nodes if n.get("id") != cid]


class UpdateNodeCommand(Command):
    """
    Update arbitrary properties of a node (pose, scale, geometry, material, static, inertial, etc.)
    with automatic inertia recalculation and optional coalescing for continuous UI sliders/gizmos.
    """

    def __init__(
        self,
        node_id: str,
        patch: Dict[str, Any],
        label: Optional[str] = None,
        coalesce_key: Optional[str] = None,
    ) -> None:
        self.node_id = node_id
        self.patch = copy.deepcopy(patch)
        self.before_snapshot: Optional[Dict[str, Any]] = None
        self.after_snapshot: Optional[Dict[str, Any]] = None
        super().__init__(
            label=label or f"Update '{node_id}'",
            coalesce_key=coalesce_key,
        )

    def execute(self, doc: SceneGraphDocument) -> None:
        node = doc.get_node(self.node_id)
        if node is None:
            return
        if self.before_snapshot is None:
            self.before_snapshot = copy.deepcopy(node)

        if self.after_snapshot is not None:
            node.clear()
            node.update(copy.deepcopy(self.after_snapshot))
            return

        if "name" in self.patch and isinstance(self.patch["name"], str):
            self.patch["name"] = doc.ensure_unique_name(
                self.patch["name"], exclude_id=self.node_id
            )

        old_size = node.get("geometry", {}).get("size", [1, 1, 1])
        old_height = float(old_size[2]) if isinstance(old_size, list) and len(old_size) >= 3 else 0.0
        old_z = float(node.get("pose", [0, 0, 0, 0, 0, 0])[2])
        was_on_ground = old_height > 0.0 and abs(old_z - old_height * 0.5) < 1e-3

        _deep_merge_dict(node, self.patch)
        if node.get("archetype") and any(
            k in self.patch for k in ("archetype_params", "material")
        ):
            rebuild_archetype_node(node)
            if was_on_ground and "pose" not in self.patch:
                new_size = node.get("geometry", {}).get("size", [1, 1, 1])
                new_h = float(new_size[2]) if isinstance(new_size, list) and len(new_size) >= 3 else old_height
                node["pose"][2] = clean_float(new_h * 0.5)

        normalize_node_in_place(node)

        # If geometry, scale, archetype_params, or inertial mode/preset/mass changed, recalculate inertia
        if any(k in self.patch for k in ("geometry", "scale", "archetype_params")) or (
            "inertial" in self.patch
            and self.patch["inertial"].get("mode", node.get("inertial", {}).get("mode"))
            in ("auto", "manual_mass")
        ):
            if node.get("type") in ("primitive", "mesh"):
                recalculate_node_inertial(node)

        self.after_snapshot = copy.deepcopy(node)

    def undo(self, doc: SceneGraphDocument) -> None:
        node = doc.get_node(self.node_id)
        if node is not None and self.before_snapshot is not None:
            node.clear()
            node.update(copy.deepcopy(self.before_snapshot))

    def merge_with(self, newer: Command) -> bool:
        if not isinstance(newer, UpdateNodeCommand):
            return False
        if newer.node_id != self.node_id or newer.after_snapshot is None:
            return False
        self.after_snapshot = copy.deepcopy(newer.after_snapshot)
        return True


class DropToGroundCommand(Command):
    """
    Drop a node along -Z so its world AABB bottom rests flush on the highest supporting object
    underneath it (table, pallet, rack shelf) or on `ground_z` (default 0.0).
    """

    def __init__(self, node_id: str, ground_z: Optional[float] = None) -> None:
        self.node_id = node_id
        self.ground_z = ground_z
        self.old_pose: Optional[List[float]] = None
        self.new_pose: Optional[List[float]] = None
        super().__init__(label=f"Drop '{node_id}' to Surface")

    def execute(self, doc: SceneGraphDocument) -> None:
        node = doc.get_node(self.node_id)
        if node is None:
            return
        if self.old_pose is None:
            self.old_pose = list(node.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
            if self.ground_z is None:
                new_z = compute_smart_drop_z(doc, self.node_id)
            else:
                geom = node.get("geometry", {})
                shape = geom.get("shape", "box")
                size = geom.get("size", [1.0, 1.0, 1.0])
                radius = float(geom.get("radius", 0.5))
                length = float(geom.get("length", 1.0))
                half_ext = compute_local_half_extents(shape, size=size, radius=radius, length=length)
                scale = node.get("scale", [1.0, 1.0, 1.0])
                new_z = compute_drop_to_ground_z(self.old_pose, half_ext, ground_z=float(self.ground_z), scale=scale)
            self.new_pose = list(self.old_pose)
            self.new_pose[2] = new_z
            self.label = f"Drop '{node.get('name')}' -> Z={new_z:.3f}m"
        node["pose"] = list(self.new_pose)

    def undo(self, doc: SceneGraphDocument) -> None:
        node = doc.get_node(self.node_id)
        if node is not None and self.old_pose is not None:
            node["pose"] = list(self.old_pose)


class UpdateWorldSettingsCommand(Command):
    """Update world-level configuration (name, sdf_target, gravity, physics, plugins)."""

    def __init__(self, world_patch: Dict[str, Any]) -> None:
        self.world_patch = copy.deepcopy(world_patch)
        self.before_world: Optional[Dict[str, Any]] = None
        super().__init__(label="Update World Settings")

    def execute(self, doc: SceneGraphDocument) -> None:
        if self.before_world is None:
            self.before_world = copy.deepcopy(doc.world)
        _deep_merge_dict(doc.world, self.world_patch)
        target = doc.world.get("sdf_target", "harmonic")
        if "sdf_version" not in self.world_patch:
            doc.world["sdf_version"] = "1.10" if target == "harmonic" else "1.7"

    def undo(self, doc: SceneGraphDocument) -> None:
        if self.before_world is not None:
            doc.world = copy.deepcopy(self.before_world)


class ExplodeCompositeCommand(Command):
    """
    Break a multi-part composite node (`node["parts"]`) into separate independent primitive nodes
    positioned in world space, with full Undo back to the single parametric composite.
    """

    def __init__(self, node_id: str) -> None:
        self.node_id = node_id
        self.original_index: int = -1
        self.original_node: Optional[Dict[str, Any]] = None
        self.spawned_nodes: List[Dict[str, Any]] = []
        super().__init__(label=f"Explode Composite '{node_id}'")

    def execute(self, doc: SceneGraphDocument) -> None:
        if self.original_node is None:
            for idx, n in enumerate(doc.nodes):
                if n.get("id") == self.node_id:
                    self.original_index = idx
                    self.original_node = copy.deepcopy(n)
                    break
            if self.original_node is None:
                return

            parts = self.original_node.get("parts", [])
            if not parts:
                return

            parent_pose = self.original_node.get("pose", [0, 0, 0, 0, 0, 0])
            px, py, pz, pr, pp, pyaw = (float(v) for v in parent_pose[:6])
            psx, psy, psz = (float(v) for v in self.original_node.get("scale", [1, 1, 1])[:3])
            rot = euler_rpy_to_rotation_matrix(pr, pp, pyaw)
            base_name = self.original_node.get("name", "part")

            for p in parts:
                lp = p.get("local_pose", [0, 0, 0, 0, 0, 0])
                lx, ly, lz = float(lp[0]) * psx, float(lp[1]) * psy, float(lp[2]) * psz
                wx = px + rot[0][0] * lx + rot[0][1] * ly + rot[0][2] * lz
                wy = py + rot[1][0] * lx + rot[1][1] * ly + rot[1][2] * lz
                wz = pz + rot[2][0] * lx + rot[2][1] * ly + rot[2][2] * lz

                psize = p.get("size", [1, 1, 1])
                scaled_size = [float(psize[0]) * psx, float(psize[1]) * psy, float(psize[2]) * psz]
                child = create_primitive_node(
                    name=doc.ensure_unique_name(f"{base_name}_{p.get('name', 'part')}"),
                    shape=p.get("shape", "box"),
                    pose=[wx, wy, wz, pr + float(lp[3]), pp + float(lp[4]), pyaw + float(lp[5])],
                    size=scaled_size,
                    radius=float(p.get("radius", 0.2)) * 0.5 * (psx + psy),
                    length=float(p.get("length", 1.0)) * psz,
                    color=p.get("color", self.original_node.get("material", {}).get("color")),
                    static=bool(self.original_node.get("static", True)),
                    material_preset=self.original_node.get("inertial", {}).get("material_preset", "concrete"),
                    folder_id=self.original_node.get("folder_id", "folder_arch"),
                )
                self.spawned_nodes.append(child)
            self.label = f"Explode '{base_name}' into {len(self.spawned_nodes)} parts"

        doc.nodes = [n for n in doc.nodes if n.get("id") != self.node_id]
        doc.nodes.extend(copy.deepcopy(self.spawned_nodes))

    def undo(self, doc: SceneGraphDocument) -> None:
        if self.original_node is None:
            return
        spawned_ids = {n["id"] for n in self.spawned_nodes}
        doc.nodes = [n for n in doc.nodes if n.get("id") not in spawned_ids]
        insert_at = min(max(self.original_index, 0), len(doc.nodes))
        doc.nodes.insert(insert_at, copy.deepcopy(self.original_node))


class AddMultipleNodesCommand(Command):
    """
    Add a batch of procedurally generated nodes (Floorplan walls, Grid Array, Spline Clones,
    Scatter Brush, or PGM wall reconstruction) in a single atomic Undo/Redo operation.
    """

    def __init__(self, nodes_list: List[Dict[str, Any]], label: str = "Generate Procedural Objects") -> None:
        self.nodes_list = [normalize_node_in_place(copy.deepcopy(n)) for n in nodes_list]
        super().__init__(label=label)

    def execute(self, doc: SceneGraphDocument) -> None:
        for n in self.nodes_list:
            n["name"] = doc.ensure_unique_name(n["name"], exclude_id=n["id"])
            doc.nodes.append(copy.deepcopy(n))

    def undo(self, doc: SceneGraphDocument) -> None:
        added_ids = {n["id"] for n in self.nodes_list}
        doc.nodes = [n for n in doc.nodes if n.get("id") not in added_ids]


class BatchUpdatePosesCommand(Command):
    """Update the poses of one or more nodes atomically (for Snap, Align, Distribute, and Multi-Gizmo tools)."""

    def __init__(self, new_poses_map: Dict[str, List[float]], label: str = "Align/Snap Objects") -> None:
        self.new_poses_map = copy.deepcopy(new_poses_map)
        self.old_poses_map: Dict[str, List[float]] = {}
        super().__init__(label=label)

    def execute(self, doc: SceneGraphDocument) -> None:
        for nid, npose in self.new_poses_map.items():
            node = doc.get_node(nid)
            if node is not None:
                if nid not in self.old_poses_map:
                    self.old_poses_map[nid] = list(node.get("pose", [0, 0, 0, 0, 0, 0]))
                node["pose"] = [clean_float(float(v), 6) for v in npose[:6]]

    def undo(self, doc: SceneGraphDocument) -> None:
        for nid, opose in self.old_poses_map.items():
            node = doc.get_node(nid)
            if node is not None:
                node["pose"] = list(opose)


class DuplicateNodesCommand(Command):
    """Duplicate multiple selected nodes (`Ctrl+D`) in a single atomic Undo/Redo operation."""

    def __init__(self, source_node_ids: List[str], offset_xy: tuple[float, float] = (0.35, 0.35)) -> None:
        self.source_node_ids = list(dict.fromkeys(source_node_ids))
        self.offset_xy = offset_xy
        self.created_nodes: List[Dict[str, Any]] = []
        super().__init__(label=f"Duplicate {len(self.source_node_ids)} object(s)")

    def execute(self, doc: SceneGraphDocument) -> None:
        if not self.created_nodes:
            for sid in self.source_node_ids:
                src = doc.get_node(sid)
                if src is None or src.get("type") == "ground_plane":
                    continue
                dup = copy.deepcopy(src)
                dup["id"] = generate_node_id(dup.get("type", "node"))
                dup["name"] = doc.ensure_unique_name(f"{src.get('name', 'object')}_copy")
                dup["locked"] = False
                pose = list(dup.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
                pose[0] = round(float(pose[0]) + self.offset_xy[0], 6)
                pose[1] = round(float(pose[1]) + self.offset_xy[1], 6)
                dup["pose"] = pose
                self.created_nodes.append(normalize_node_in_place(dup))
        for cn in self.created_nodes:
            doc.nodes.append(copy.deepcopy(cn))

    def undo(self, doc: SceneGraphDocument) -> None:
        cids = {n["id"] for n in self.created_nodes}
        doc.nodes = [n for n in doc.nodes if n.get("id") not in cids]


class GroupNodesCommand(Command):
    """
    Merge 2 or more selected nodes into a single composite group node (`custom_group`)
    whose origin sits at the combined world AABB center.
    Full Undo restores the original independent nodes at their exact indices.
    """

    def __init__(self, node_ids: List[str], group_name: str = "grupo_objetos") -> None:
        self.node_ids = list(dict.fromkeys(node_ids))
        self.group_name = group_name
        self.removed_entries: List[tuple[int, Dict[str, Any]]] = []
        self.group_node: Optional[Dict[str, Any]] = None
        super().__init__(label=f"Group {len(self.node_ids)} Objects")

    def execute(self, doc: SceneGraphDocument) -> None:
        if self.group_node is None:
            id_set = set(self.node_ids)
            members: List[Dict[str, Any]] = []
            for idx, n in enumerate(doc.nodes):
                if n.get("id") in id_set and n.get("type") not in ("ground_plane", "light"):
                    self.removed_entries.append((idx, copy.deepcopy(n)))
                    members.append(n)

            if len(members) < 2:
                self.removed_entries.clear()
                return

            # Compute combined world AABB across all member nodes
            min_x, min_y, min_z = 1e9, 1e9, 1e9
            max_x, max_y, max_z = -1e9, -1e9, -1e9
            for m in members:
                bmin, bmax = get_node_world_aabb(m)
                min_x = min(min_x, bmin[0])
                min_y = min(min_y, bmin[1])
                min_z = min(min_z, bmin[2])
                max_x = max(max_x, bmax[0])
                max_y = max(max_y, bmax[1])
                max_z = max(max_z, bmax[2])

            cx = (min_x + max_x) * 0.5
            cy = (min_y + max_y) * 0.5
            cz = (min_z + max_z) * 0.5
            bbox_size = [
                max(max_x - min_x, 0.1),
                max(max_y - min_y, 0.1),
                max(max_z - min_z, 0.1),
            ]

            combined_parts: List[Dict[str, Any]] = []
            for m_idx, m in enumerate(members):
                mp = m.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                mx, my, mz, mr, mpitch, myaw = (float(v) for v in mp[:6])
                ms = m.get("scale", [1.0, 1.0, 1.0])
                sx, sy, sz = (float(v) for v in ms[:3])
                m_color = m.get("material", {}).get("color", [0.75, 0.78, 0.82, 1.0])
                m_parts = m.get("parts")

                if isinstance(m_parts, list) and len(m_parts) > 0:
                    rot = euler_rpy_to_rotation_matrix(mr, mpitch, myaw)
                    for p_idx, p in enumerate(m_parts):
                        lp = p.get("local_pose", [0, 0, 0, 0, 0, 0])
                        lx, ly, lz = float(lp[0]) * sx, float(lp[1]) * sy, float(lp[2]) * sz
                        wx = mx + rot[0][0] * lx + rot[0][1] * ly + rot[0][2] * lz
                        wy = my + rot[1][0] * lx + rot[1][1] * ly + rot[1][2] * lz
                        wz = mz + rot[2][0] * lx + rot[2][1] * ly + rot[2][2] * lz
                        psize = p.get("size", [1.0, 1.0, 1.0])
                        part_dict: Dict[str, Any] = {
                            "name": f"{m.get('name', 'obj')}_{p.get('name', p_idx + 1)}",
                            "shape": p.get("shape", "box"),
                            "local_pose": [
                                clean_float(wx - cx, 6),
                                clean_float(wy - cy, 6),
                                clean_float(wz - cz, 6),
                                clean_float(mr + float(lp[3]), 6),
                                clean_float(mpitch + float(lp[4]), 6),
                                clean_float(myaw + float(lp[5]), 6),
                            ],
                            "size": [
                                clean_float(float(psize[0]) * sx, 6),
                                clean_float(float(psize[1]) * sy, 6),
                                clean_float(float(psize[2]) * sz, 6),
                            ],
                            "radius": clean_float(float(p.get("radius", 0.2)) * 0.5 * (sx + sy), 6),
                            "length": clean_float(float(p.get("length", 1.0)) * sz, 6),
                            "color": list(p.get("color", m_color)),
                        }
                        if p.get("shape") == "mesh" and p.get("mesh_filename"):
                            pm_scale = p.get("mesh_scale", [1.0, 1.0, 1.0])
                            part_dict["mesh_filename"] = p["mesh_filename"]
                            part_dict["mesh_uri"] = p.get("mesh_uri", f"meshes/{p['mesh_filename']}")
                            part_dict["mesh_scale"] = [
                                clean_float(float(pm_scale[0]) * sx, 6),
                                clean_float(float(pm_scale[1]) * sy, 6),
                                clean_float(float(pm_scale[2]) * sz, 6),
                            ]
                            part_dict["up_axis"] = p.get("up_axis", "Z_UP")
                        combined_parts.append(part_dict)
                else:
                    geom = m.get("geometry", {})
                    gsize = geom.get("size", [1.0, 1.0, 1.0])
                    is_mesh_node = m.get("type") == "mesh" and bool(m.get("mesh", {}).get("filename"))
                    part_dict = {
                        "name": f"{m.get('name', f'part_{m_idx + 1}')}",
                        "shape": "mesh" if is_mesh_node else geom.get("shape", "box"),
                        "local_pose": [
                            clean_float(mx - cx, 6),
                            clean_float(my - cy, 6),
                            clean_float(mz - cz, 6),
                            clean_float(mr, 6),
                            clean_float(mpitch, 6),
                            clean_float(myaw, 6),
                        ],
                        "size": [
                            clean_float(float(gsize[0]) * sx, 6),
                            clean_float(float(gsize[1]) * sy, 6),
                            clean_float(float(gsize[2]) * sz, 6),
                        ],
                        "radius": clean_float(float(geom.get("radius", 0.5)) * 0.5 * (sx + sy), 6),
                        "length": clean_float(float(geom.get("length", 1.0)) * sz, 6),
                        "color": list(m_color),
                    }
                    if is_mesh_node:
                        minfo = m["mesh"]
                        u_scale = float(minfo.get("unit_scale", 1.0))
                        part_dict["mesh_filename"] = minfo["filename"]
                        part_dict["mesh_uri"] = minfo.get("uri", f"meshes/{minfo['filename']}")
                        part_dict["mesh_scale"] = [
                            clean_float(u_scale * sx, 6),
                            clean_float(u_scale * sy, 6),
                            clean_float(u_scale * sz, 6),
                        ]
                        part_dict["up_axis"] = minfo.get("up_axis", "Z_UP")
                    combined_parts.append(part_dict)

            first = members[0]
            all_static = all(bool(m.get("static", True)) for m in members)
            gnode = create_primitive_node(
                name=doc.ensure_unique_name(self.group_name),
                shape="box",
                pose=[clean_float(cx, 6), clean_float(cy, 6), clean_float(cz, 6), 0.0, 0.0, 0.0],
                size=[clean_float(bbox_size[0], 5), clean_float(bbox_size[1], 5), clean_float(bbox_size[2], 5)],
                color=list(first.get("material", {}).get("color", [0.35, 0.65, 0.90, 1.0])),
                static=all_static,
                material_preset=first.get("inertial", {}).get("material_preset", "wood"),
                folder_id=first.get("folder_id", "folder_props"),
                node_id=generate_node_id("group"),
            )
            gnode["archetype"] = "custom_group"
            gnode["archetype_params"] = {}
            gnode["parts"] = combined_parts
            self.group_node = normalize_node_in_place(gnode)
            self.label = f"Group {len(members)} objects -> '{self.group_node['name']}'"

        if self.group_node is None:
            return

        removed_ids = {entry[1]["id"] for entry in self.removed_entries}
        doc.nodes = [n for n in doc.nodes if n.get("id") not in removed_ids]
        doc.nodes.append(copy.deepcopy(self.group_node))

    def undo(self, doc: SceneGraphDocument) -> None:
        if self.group_node is None:
            return
        gid = self.group_node["id"]
        doc.nodes = [n for n in doc.nodes if n.get("id") != gid]
        for idx, saved_node in sorted(self.removed_entries, key=lambda x: x[0]):
            insert_at = min(idx, len(doc.nodes))
            doc.nodes.insert(insert_at, copy.deepcopy(saved_node))


class SwapNodesOrderCommand(Command):
    """Swap the order of two nodes in `doc.nodes` (used for Nav2 waypoint route ordering)."""

    def __init__(self, node_id_a: str, node_id_b: str, label: str = "Reorder Waypoints") -> None:
        self.node_id_a = node_id_a
        self.node_id_b = node_id_b
        super().__init__(label=label)

    def _swap(self, doc: SceneGraphDocument) -> None:
        idx_a = next((i for i, n in enumerate(doc.nodes) if n.get("id") == self.node_id_a), -1)
        idx_b = next((i for i, n in enumerate(doc.nodes) if n.get("id") == self.node_id_b), -1)
        if idx_a != -1 and idx_b != -1 and idx_a != idx_b:
            doc.nodes[idx_a], doc.nodes[idx_b] = doc.nodes[idx_b], doc.nodes[idx_a]

    def execute(self, doc: SceneGraphDocument) -> None:
        self._swap(doc)

    def undo(self, doc: SceneGraphDocument) -> None:
        self._swap(doc)





