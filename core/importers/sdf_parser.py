"""
Round-Trip SDF / World Parser (`core/importers/sdf_parser.py`).

Imports an existing `.sdf` or `.world` XML file (including dirty files saved from Gazebo GUI),
strips all residual dynamic state tags (`<state>`, `<sim_time>`, `<iterations>`, etc.),
and reconstructs a clean `SceneGraphDocument` ready for visual 3D editing.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from typing import Any, Dict, List, Optional

from core.math_physics.transforms import clean_float
from core.schema.scene_graph import (
    SceneGraphDocument,
    create_ground_plane_node,
    create_light_node,
    create_primitive_node,
    generate_node_id,
    normalize_node_in_place,
)


def _parse_float_list(text: Optional[str], expected_len: int, default: List[float]) -> List[float]:
    if not text:
        return list(default)
    parts = text.strip().split()
    try:
        vals = [clean_float(float(x)) for x in parts[:expected_len]]
        while len(vals) < expected_len:
            vals.append(default[len(vals)])
        return vals
    except ValueError:
        return list(default)


def parse_sdf_to_scene_graph(xml_content: str) -> SceneGraphDocument:
    """
    Parse an SDFormat XML string (`.sdf` or `.world`), discard dynamic residual `<state>` blocks,
    and reconstruct a normalized `SceneGraphDocument`.
    """
    root = ET.fromstring(xml_content)
    if root.tag != "sdf":
        raise ValueError(f"Root XML element must be <sdf>, got <{root.tag}>.")

    sdf_version = root.attrib.get("version", "1.10")
    sdf_target = "classic" if sdf_version in ("1.5", "1.6", "1.7") else "harmonic"

    world_el = root.find("world")
    if world_el is None:
        # Handle standalone model SDF by wrapping it in a default world
        world_name = "imported_model_world"
        doc = SceneGraphDocument.create_default(world_name)
        doc.nodes = [n for n in doc.nodes if n.get("type") in ("light", "ground_plane")]
        for m_el in root.findall("model"):
            parsed_node = _parse_model_element(m_el)
            if parsed_node:
                doc.nodes.append(parsed_node)
        return doc

    world_name = world_el.attrib.get("name", "imported_world")
    doc = SceneGraphDocument.create_default(world_name)
    doc.world["sdf_target"] = sdf_target
    doc.world["sdf_version"] = sdf_version

    grav_el = world_el.find("gravity")
    if grav_el is not None and grav_el.text:
        doc.world["gravity"] = _parse_float_list(grav_el.text, 3, [0.0, 0.0, -9.81])

    phys_el = world_el.find("physics")
    if phys_el is not None:
        doc.world["physics"]["name"] = phys_el.attrib.get("name", "1ms")
        step_el = phys_el.find("max_step_size")
        if step_el is not None and step_el.text:
            try:
                doc.world["physics"]["max_step_size"] = float(step_el.text.strip())
            except ValueError:
                pass

    # Clear default starter nodes; we will populate from the imported <world>
    doc.nodes = []

    # 1. Parse Lights
    for light_el in world_el.findall("light"):
        lname = light_el.attrib.get("name", "light")
        ltype = light_el.attrib.get("type", "point")
        pose = _parse_float_list(light_el.findtext("pose"), 6, [0.0, 0.0, 6.0, 0.0, 0.0, 0.0])
        diffuse = _parse_float_list(light_el.findtext("diffuse"), 4, [0.85, 0.85, 0.82, 1.0])
        specular = _parse_float_list(light_el.findtext("specular"), 4, [0.25, 0.25, 0.25, 1.0])
        direction = _parse_float_list(light_el.findtext("direction"), 3, [-0.5, 0.1, -0.9])
        shadows_txt = (light_el.findtext("cast_shadows") or "true").strip().lower()
        cast_shadows = shadows_txt in ("true", "1")
        doc.nodes.append(
            create_light_node(
                name=doc.ensure_unique_name(lname),
                light_type=ltype,
                pose=pose,
                direction=direction,
                diffuse=diffuse,
                specular=specular,
                cast_shadows=cast_shadows,
            )
        )

    # 2. Parse Models (intentionally ignoring any <state> tags!)
    for model_el in world_el.findall("model"):
        parsed_node = _parse_model_element(model_el)
        if parsed_node:
            parsed_node["name"] = doc.ensure_unique_name(parsed_node["name"])
            doc.nodes.append(parsed_node)

    # 3. Parse <include> blocks (Gazebo Fuel / Workspace models)
    for inc_el in world_el.findall("include"):
        uri_text = (inc_el.findtext("uri") or "").strip()
        if not uri_text:
            continue
        inc_name = (inc_el.findtext("name") or uri_text.rstrip("/").split("/")[-1] or "included_model").strip()
        inc_pose = _parse_float_list(inc_el.findtext("pose"), 6, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        static_txt = (inc_el.findtext("static") or "true").strip().lower()
        is_static = static_txt in ("true", "1")

        inc_node: Dict[str, Any] = {
            "id": generate_node_id("include"),
            "name": doc.ensure_unique_name(inc_name),
            "type": "include",
            "folder_id": "folder_props",
            "visible": True,
            "locked": False,
            "static": is_static,
            "pose": inc_pose,
            "scale": [1.0, 1.0, 1.0],
            "include": {
                "uri": uri_text,
            },
            "geometry": {
                "shape": "box",
                "size": [1.0, 1.0, 1.0],
            },
            "material": {
                "color": [0.30, 0.62, 0.90, 1.0],
            },
        }
        doc.nodes.append(normalize_node_in_place(inc_node))

    # Ensure at least one light exists so the 3D scene isn't dark
    if not any(n.get("type") == "light" for n in doc.nodes):
        doc.nodes.insert(0, create_light_node(name="sun", light_type="directional"))

    return doc


def _parse_model_element(model_el: ET.Element) -> Optional[Dict[str, Any]]:
    mname = model_el.attrib.get("name", "model")
    pose = _parse_float_list(model_el.findtext("pose"), 6, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    static_txt = (model_el.findtext("static") or "false").strip().lower()
    is_static = static_txt in ("true", "1")

    link_el = model_el.find("link")
    if link_el is None:
        return None

    visuals = link_el.findall("visual")
    collisions = link_el.findall("collision")
    geom_sources = visuals if visuals else collisions
    if not geom_sources:
        return None

    first_geom = geom_sources[0].find("geometry")
    if first_geom is None:
        return None

    # Check if it's a ground plane
    plane_el = first_geom.find("plane")
    if plane_el is not None and len(geom_sources) == 1:
        psize = _parse_float_list(plane_el.findtext("size"), 2, [100.0, 100.0])
        return create_ground_plane_node(name=mname, size=psize)

    # Check if it's a <mesh> model
    mesh_el = first_geom.find("mesh")
    if mesh_el is not None:
        uri_val = (mesh_el.findtext("uri") or "").strip()
        scale_val = _parse_float_list(mesh_el.findtext("scale"), 3, [1.0, 1.0, 1.0])
        color = _extract_visual_color(geom_sources[0])
        node: Dict[str, Any] = {
            "id": generate_node_id("mesh"),
            "name": mname,
            "type": "mesh",
            "folder_id": "folder_props",
            "visible": True,
            "locked": False,
            "static": is_static,
            "pose": pose,
            "scale": scale_val,
            "mesh": {
                "uri": uri_val,
                "filename": uri_val.split("/")[-1],
                "unit": "m",
                "up_axis": "Z_UP",
            },
            "geometry": {"shape": "box", "size": [1.0, 1.0, 1.0]},
            "material": {"color": color, "roughness": 0.45, "metalness": 0.2},
            "collision": {"enabled": len(collisions) > 0, "strategy": "aabb", "mu1": 0.7, "mu2": 0.7},
            "inertial": _extract_inertial(link_el),
        }
        return normalize_node_in_place(node)

    # Multi-part composite model (more than 1 visual in the link)
    if len(visuals) > 1:
        parts: List[Dict[str, Any]] = []
        for idx, vis_el in enumerate(visuals):
            v_geom = vis_el.find("geometry")
            if v_geom is None:
                continue
            v_pose = _parse_float_list(vis_el.findtext("pose"), 6, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            v_color = _extract_visual_color(vis_el)
            shape, size, radius, length = _extract_primitive_params(v_geom)
            parts.append(
                {
                    "name": vis_el.attrib.get("name", f"part_{idx + 1}"),
                    "shape": shape,
                    "local_pose": v_pose,
                    "size": size,
                    "radius": radius,
                    "length": length,
                    "color": v_color,
                }
            )
        color0 = parts[0]["color"] if parts else [0.75, 0.78, 0.82, 1.0]
        comp_node = create_primitive_node(
            name=mname,
            shape="box",
            pose=pose,
            size=[1.5, 1.5, 1.5],
            color=color0,
            static=is_static,
            folder_id="folder_arch" if is_static else "folder_props",
        )
        comp_node["parts"] = parts
        comp_node["inertial"] = _extract_inertial(link_el)
        return normalize_node_in_place(comp_node)

    # Single primitive model
    shape, size, radius, length = _extract_primitive_params(first_geom)
    color = _extract_visual_color(geom_sources[0])
    prim_node = create_primitive_node(
        name=mname,
        shape=shape,
        pose=pose,
        size=size,
        radius=radius,
        length=length,
        color=color,
        static=is_static,
        folder_id="folder_arch" if is_static else "folder_props",
    )
    if not is_static:
        prim_node["inertial"] = _extract_inertial(link_el)
    return normalize_node_in_place(prim_node)


def _extract_primitive_params(geom_el: ET.Element) -> tuple[str, List[float], float, float]:
    box_el = geom_el.find("box")
    if box_el is not None:
        sz = _parse_float_list(box_el.findtext("size"), 3, [1.0, 1.0, 1.0])
        return "box", sz, 0.5, 1.0
    cyl_el = geom_el.find("cylinder")
    if cyl_el is not None:
        r = float(cyl_el.findtext("radius") or 0.5)
        l = float(cyl_el.findtext("length") or 1.0)
        return "cylinder", [r * 2, r * 2, l], r, l
    sph_el = geom_el.find("sphere")
    if sph_el is not None:
        r = float(sph_el.findtext("radius") or 0.5)
        return "sphere", [r * 2, r * 2, r * 2], r, r * 2
    return "box", [1.0, 1.0, 1.0], 0.5, 1.0


def _extract_visual_color(vis_el: ET.Element) -> List[float]:
    mat_el = vis_el.find("material")
    if mat_el is not None:
        diff_text = mat_el.findtext("diffuse") or mat_el.findtext("ambient")
        if diff_text:
            return _parse_float_list(diff_text, 4, [0.25, 0.52, 0.85, 1.0])
    return [0.25, 0.52, 0.85, 1.0]


def _extract_inertial(link_el: ET.Element) -> Dict[str, Any]:
    in_el = link_el.find("inertial")
    if in_el is None:
        return {"mode": "auto", "material_preset": "wood", "center_of_mass": [0.0, 0.0, 0.0]}
    try:
        mass = float(in_el.findtext("mass") or 1.0)
        imat = in_el.find("inertia")
        if imat is not None:
            return {
                "mode": "manual_mass",
                "material_preset": "wood",
                "center_of_mass": [0.0, 0.0, 0.0],
                "mass": clean_float(mass),
                "ixx": clean_float(float(imat.findtext("ixx") or 0.1)),
                "iyy": clean_float(float(imat.findtext("iyy") or 0.1)),
                "izz": clean_float(float(imat.findtext("izz") or 0.1)),
                "ixy": clean_float(float(imat.findtext("ixy") or 0.0)),
                "ixz": clean_float(float(imat.findtext("ixz") or 0.0)),
                "iyz": clean_float(float(imat.findtext("iyz") or 0.0)),
            }
    except ValueError:
        pass
    return {"mode": "auto", "material_preset": "wood", "center_of_mass": [0.0, 0.0, 0.0]}
