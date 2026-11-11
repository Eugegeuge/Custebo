#!/usr/bin/env python3
"""
Custebo CLI Validator — Scene Graph JSON & Gazebo SDF/XML Linter ("Doctor Gazebo").

Usage:
  python3 tools/validate_sdf.py <file1.sdf|file2.json> [...]
  python3 tools/validate_sdf.py --self-test
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, List, Optional, Tuple


SUPPORTED_SDF_VERSIONS = {"1.6", "1.7", "1.8", "1.9", "1.10", "1.11"}
FORBIDDEN_DYNAMIC_TAGS = {"state", "iterations", "sim_time", "real_time", "wall_time"}
VALID_NODE_TYPES = {
    "primitive",
    "mesh",
    "light",
    "include",
    "group",
    "sensor",
    "actor",
    "ground_plane",
}
VALID_PRIMITIVE_SHAPES = {"box", "cylinder", "sphere", "plane", "capsule"}
VALID_COLLISION_STRATEGIES = {
    "primitive",
    "aabb",
    "cylinder",
    "convex_hull",
    "decimated_mesh",
    "none",
}


@dataclass
class ValidationIssue:
    severity: str  # "ERROR" | "WARNING"
    code: str
    message: str
    location: str = ""

    def __str__(self) -> str:
        loc = f" [{self.location}]" if self.location else ""
        return f"[{self.severity}] ({self.code}){loc}: {self.message}"


@dataclass
class ValidationReport:
    target: str
    issues: List[ValidationIssue] = field(default_factory=list)
    gz_cli_checked: bool = False

    @property
    def is_valid(self) -> bool:
        return not any(i.severity == "ERROR" for i in self.issues)

    def add_error(self, code: str, message: str, location: str = "") -> None:
        self.issues.append(ValidationIssue("ERROR", code, message, location))

    def add_warning(self, code: str, message: str, location: str = "") -> None:
        self.issues.append(ValidationIssue("WARNING", code, message, location))


def is_positive_definite_inertia(
    ixx: float,
    iyy: float,
    izz: float,
    ixy: float = 0.0,
    ixz: float = 0.0,
    iyz: float = 0.0,
    eps: float = 1e-9,
) -> Tuple[bool, str]:
    """
    Verify that a 3x3 symmetric inertia tensor is physically valid:
    1. Sylvester's criterion for positive-definiteness (leading principal minors > 0)
    2. Triangle inequality on diagonal elements (ixx + iyy >= izz, etc.)
    """
    vals = [ixx, iyy, izz, ixy, ixz, iyz]
    if any(math.isnan(v) or math.isinf(v) for v in vals):
        return False, "Inertia tensor contains NaN or Infinite values."

    if ixx <= eps or iyy <= eps or izz <= eps:
        return (
            False,
            f"Principal diagonal moments must be > 0 (got ixx={ixx}, iyy={iyy}, izz={izz}).",
        )

    # Leading principal minor 2x2:
    minor2 = ixx * iyy - ixy * ixy
    if minor2 <= eps:
        return False, f"2x2 principal minor (ixx*iyy - ixy^2 = {minor2:.6g}) must be > 0."

    # Determinant of 3x3 symmetric matrix:
    det = (
        ixx * (iyy * izz - iyz * iyz)
        - ixy * (ixy * izz - iyz * ixz)
        + ixz * (ixy * iyz - iyy * ixz)
    )
    if det <= eps:
        return False, f"Inertia matrix determinant ({det:.6g}) must be > 0."

    # Triangle inequalities (with small numerical tolerance):
    tol = 1e-7
    if (ixx + iyy + tol < izz) or (ixx + izz + tol < iyy) or (iyy + izz + tol < ixx):
        return (
            False,
            f"Triangle inequality violated: ixx={ixx}, iyy={iyy}, izz={izz} "
            "(sum of any two moments must be >= the third).",
        )

    return True, "OK"


def validate_pose_string(pose_text: str, location: str, report: ValidationReport) -> None:
    if re.search(r"(^|\s)-0(\.0+)?(\s|$)", pose_text):
        report.add_error(
            "SDF_NEGATIVE_ZERO",
            f"Pose contains dirty '-0.0' formatting: '{pose_text}'",
            location,
        )
    parts = pose_text.strip().split()
    if len(parts) not in (6, 7):
        report.add_error(
            "SDF_INVALID_POSE_LEN",
            f"Expected 6 (x y z r p y) or 7 values in <pose>, got {len(parts)}: '{pose_text}'",
            location,
        )
        return
    try:
        nums = [float(p) for p in parts]
        if any(math.isnan(n) or math.isinf(n) for n in nums):
            report.add_error("SDF_NAN_POSE", f"Pose contains NaN/Inf: '{pose_text}'", location)
    except ValueError:
        report.add_error(
            "SDF_NON_NUMERIC_POSE", f"Pose contains non-numeric tokens: '{pose_text}'", location
        )


def find_gz_binary() -> Optional[str]:
    gz_in_path = shutil.which("gz")
    if gz_in_path:
        return gz_in_path
    jazzy_gz = "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz"
    if os.path.isfile(jazzy_gz) and os.access(jazzy_gz, os.X_OK):
        return jazzy_gz
    return None


def validate_sdf_content(xml_content: str, source_name: str = "<memory>") -> ValidationReport:
    report = ValidationReport(target=source_name)

    try:
        root = ET.fromstring(xml_content)
    except ET.ParseError as exc:
        report.add_error("XML_PARSE_ERROR", f"Malformed XML: {exc}", source_name)
        return report

    if root.tag != "sdf":
        report.add_error("SDF_ROOT_TAG", f"Root tag must be <sdf>, got <{root.tag}>", source_name)
        return report

    version = root.attrib.get("version", "")
    if version not in SUPPORTED_SDF_VERSIONS:
        report.add_error(
            "SDF_UNSUPPORTED_VERSION",
            f"Unsupported or missing SDF version '{version}'. Supported: {sorted(SUPPORTED_SDF_VERSIONS)}",
            "<sdf>",
        )

    # Check for forbidden dynamic residual state tags ("Zero Garbage Code" rule)
    for elem in root.iter():
        if elem.tag in FORBIDDEN_DYNAMIC_TAGS:
            report.add_error(
                "SDF_RESIDUAL_STATE",
                f"Forbidden residual dynamic state tag <{elem.tag}> found in exported SDF.",
                f"<{elem.tag}>",
            )
        if elem.tag == "pose" and elem.text:
            validate_pose_string(elem.text, "<pose>", report)
        if elem.tag == "uri" and elem.text:
            uri_val = elem.text.strip()
            if uri_val.startswith("/") or re.match(r"^[A-Za-z]:\\", uri_val):
                report.add_warning(
                    "DOCTOR_ABSOLUTE_URI",
                    f"Absolute filesystem path in <uri>: '{uri_val}'. Use model://, package://, or relative package paths.",
                    "<uri>",
                )

    worlds = root.findall("world")
    if not worlds:
        models_at_root = root.findall("model")
        if not models_at_root:
            report.add_error(
                "SDF_MISSING_WORLD_OR_MODEL",
                "SDF must contain at least one <world> or <model> element.",
                "<sdf>",
            )

    for world in worlds:
        wname = world.attrib.get("name", "").strip()
        if not wname:
            report.add_error("SDF_UNNAMED_WORLD", "<world> is missing a valid 'name' attribute.", "<world>")

        seen_model_names = set()
        for model in world.findall("model"):
            mname = model.attrib.get("name", "").strip()
            loc = f"model[@name='{mname or '?'}']"
            if not mname:
                report.add_error("SDF_UNNAMED_MODEL", "<model> is missing 'name' attribute.", loc)
            elif mname in seen_model_names:
                report.add_error("SDF_DUPLICATE_MODEL_NAME", f"Duplicate model name '{mname}'.", loc)
            else:
                seen_model_names.add(mname)

            static_el = model.find("static")
            is_static = (
                static_el is not None
                and static_el.text is not None
                and static_el.text.strip().lower() in ("true", "1")
            )

            links = model.findall("link")
            if not links:
                report.add_error("SDF_MODEL_NO_LINKS", f"Model '{mname}' has no <link> elements.", loc)

            for link in links:
                lname = link.attrib.get("name", "link")
                link_loc = f"{loc}/link[@name='{lname}']"
                if not is_static:
                    inertial = link.find("inertial")
                    if inertial is None:
                        report.add_error(
                            "DOCTOR_MISSING_INERTIAL",
                            f"Dynamic model '{mname}' (link '{lname}') is missing <inertial>.",
                            link_loc,
                        )
                        continue
                    mass_el = inertial.find("mass")
                    if mass_el is None or not mass_el.text:
                        report.add_error(
                            "DOCTOR_MISSING_MASS",
                            f"Dynamic model '{mname}' (link '{lname}') is missing <mass>.",
                            link_loc,
                        )
                    else:
                        try:
                            mass_val = float(mass_el.text.strip())
                            if mass_val <= 0.0 or math.isnan(mass_val) or math.isinf(mass_val):
                                report.add_error(
                                    "DOCTOR_INVALID_MASS",
                                    f"Dynamic model '{mname}' has non-positive mass ({mass_val} kg).",
                                    link_loc,
                                )
                        except ValueError:
                            report.add_error(
                                "DOCTOR_NON_NUMERIC_MASS",
                                f"Invalid mass value '{mass_el.text}' in '{mname}'.",
                                link_loc,
                            )

                    inertia_el = inertial.find("inertia")
                    if inertia_el is None:
                        report.add_error(
                            "DOCTOR_MISSING_INERTIA_MATRIX",
                            f"Dynamic model '{mname}' (link '{lname}') is missing <inertia> tensor.",
                            link_loc,
                        )
                    else:
                        try:
                            ixx = float(inertia_el.findtext("ixx", "0"))
                            iyy = float(inertia_el.findtext("iyy", "0"))
                            izz = float(inertia_el.findtext("izz", "0"))
                            ixy = float(inertia_el.findtext("ixy", "0"))
                            ixz = float(inertia_el.findtext("ixz", "0"))
                            iyz = float(inertia_el.findtext("iyz", "0"))
                            ok, reason = is_positive_definite_inertia(ixx, iyy, izz, ixy, ixz, iyz)
                            if not ok:
                                report.add_error("DOCTOR_INVALID_INERTIA_TENSOR", reason, link_loc)
                        except ValueError as exc:
                            report.add_error(
                                "DOCTOR_NON_NUMERIC_INERTIA",
                                f"Non-numeric inertia component in '{mname}': {exc}",
                                link_loc,
                            )

    # Run native `gz sdf -k` if available and no XML parse errors occurred
    gz_bin = find_gz_binary()
    if gz_bin and report.is_valid:
        with tempfile.NamedTemporaryFile("w", suffix=".sdf", delete=False, encoding="utf-8") as tmp:
            tmp.write(xml_content)
            tmp_path = tmp.name
        try:
            proc = subprocess.run(
                [gz_bin, "sdf", "-k", tmp_path],
                capture_output=True,
                text=True,
                timeout=10,
            )
            report.gz_cli_checked = True
            if proc.returncode != 0:
                stderr_out = (proc.stderr or proc.stdout).strip()
                report.add_error(
                    "GZ_SDF_CLI_CHECK_FAILED",
                    f"`gz sdf -k` reported error: {stderr_out}",
                    source_name,
                )
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)

    return report


def validate_scene_graph_dict(data: dict[str, Any], source_name: str = "<memory>") -> ValidationReport:
    report = ValidationReport(target=source_name)

    if not isinstance(data, dict):
        report.add_error("SG_ROOT_NOT_OBJECT", "Scene Graph root must be a JSON object.", source_name)
        return report

    required_keys = {"schema_version", "world", "nodes"}
    missing = required_keys - set(data.keys())
    if missing:
        report.add_error(
            "SG_MISSING_ROOT_KEYS",
            f"Scene Graph is missing required root keys: {sorted(missing)}",
            source_name,
        )
        return report

    nodes = data.get("nodes", [])
    if not isinstance(nodes, list):
        report.add_error("SG_NODES_NOT_LIST", "'nodes' must be a list of node objects.", "nodes")
        return report

    seen_ids = set()
    for idx, node in enumerate(nodes):
        loc = f"nodes[{idx}]"
        if not isinstance(node, dict):
            report.add_error("SG_NODE_NOT_OBJECT", "Node entry must be a JSON object.", loc)
            continue

        nid = node.get("id")
        nname = node.get("name")
        ntype = node.get("type")
        if not nid or not isinstance(nid, str):
            report.add_error("SG_NODE_MISSING_ID", "Node is missing a valid string 'id'.", loc)
        elif nid in seen_ids:
            report.add_error("SG_DUPLICATE_NODE_ID", f"Duplicate node id '{nid}'.", loc)
        else:
            seen_ids.add(nid)

        if not nname or not isinstance(nname, str):
            report.add_error("SG_NODE_MISSING_NAME", f"Node '{nid}' is missing 'name'.", loc)

        if ntype not in VALID_NODE_TYPES:
            report.add_error(
                "SG_INVALID_NODE_TYPE",
                f"Invalid node type '{ntype}'. Valid: {sorted(VALID_NODE_TYPES)}",
                loc,
            )

        pose = node.get("pose")
        if not isinstance(pose, list) or len(pose) != 6:
            report.add_error(
                "SG_INVALID_POSE",
                f"Node '{nname}' pose must be [x, y, z, roll, pitch, yaw] (6 floats).",
                loc,
            )
        else:
            if not all(isinstance(v, (int, float)) and math.isfinite(float(v)) for v in pose):
                report.add_error(
                    "SG_NON_FINITE_POSE",
                    f"Node '{nname}' pose contains non-finite numbers: {pose}",
                    loc,
                )

        scale = node.get("scale", [1.0, 1.0, 1.0])
        if (
            not isinstance(scale, list)
            or len(scale) != 3
            or not all(isinstance(v, (int, float)) and float(v) > 0 for v in scale)
        ):
            report.add_error(
                "SG_INVALID_SCALE",
                f"Node '{nname}' scale must be 3 positive numbers [sx, sy, sz], got {scale}",
                loc,
            )

        is_static = node.get("static", True)
        if not is_static and ntype in ("primitive", "mesh"):
            inertial = node.get("inertial", {})
            mass = float(inertial.get("mass", 0.0)) if isinstance(inertial, dict) else 0.0
            if mass <= 0.0:
                report.add_error(
                    "SG_DYNAMIC_ZERO_MASS",
                    f"Dynamic node '{nname}' has static=False but mass={mass} <= 0.",
                    loc,
                )

    return report


def run_self_test() -> int:
    print("Running Custebo SDF & Scene Graph Validator self-tests...")

    # 1. Valid Gazebo Harmonic SDF (1.10)
    valid_harmonic_sdf = """<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="custebo_test_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
      </link>
    </model>
    <model name="dynamic_crate">
      <static>false</static>
      <pose>1.5 2.0 0.5 0 0 0.785398</pose>
      <link name="link">
        <inertial>
          <mass>12.0</mass>
          <inertia>
            <ixx>2.0</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>2.0</iyy>
            <iyz>0.0</iyz>
            <izz>2.0</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>1.0 1.0 1.0</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.0 1.0 1.0</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
"""
    rep_ok = validate_sdf_content(valid_harmonic_sdf, "valid_harmonic.sdf")
    assert rep_ok.is_valid, f"Expected valid SDF to pass, got: {[str(i) for i in rep_ok.issues]}"

    # 2. Invalid SDF with dirty <state> and broken inertia (violates triangle inequality)
    dirty_broken_sdf = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="dirty_world">
    <state world_name="dirty_world">
      <sim_time>123 456</sim_time>
    </state>
    <model name="bad_box">
      <static>false</static>
      <pose>0 0 -0.0 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>5.0</mass>
          <inertia>
            <ixx>1.0</ixx>
            <iyy>1.0</iyy>
            <izz>10.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
"""
    rep_bad = validate_sdf_content(dirty_broken_sdf, "dirty_broken.sdf")
    assert not rep_bad.is_valid, "Expected dirty/broken SDF to fail validation!"
    error_codes = {i.code for i in rep_bad.issues}
    assert "SDF_RESIDUAL_STATE" in error_codes, f"Missing SDF_RESIDUAL_STATE in {error_codes}"
    assert "SDF_NEGATIVE_ZERO" in error_codes, f"Missing SDF_NEGATIVE_ZERO in {error_codes}"
    assert "DOCTOR_INVALID_INERTIA_TENSOR" in error_codes, f"Missing DOCTOR_INVALID_INERTIA_TENSOR in {error_codes}"

    # 3. Valid Scene Graph JSON test
    sample_sg = {
        "schema_version": "1.0.0",
        "world": {"name": "warehouse_world", "sdf_target": "harmonic", "sdf_version": "1.10"},
        "nodes": [
            {
                "id": "node_box_1",
                "name": "pallet_box",
                "type": "primitive",
                "pose": [1.0, 2.0, 0.5, 0.0, 0.0, 1.570796],
                "scale": [1.0, 1.0, 1.0],
                "static": False,
                "inertial": {"mass": 10.0, "ixx": 1.66, "iyy": 1.66, "izz": 1.66},
            }
        ],
    }
    rep_sg = validate_scene_graph_dict(sample_sg, "sample_scene_graph.json")
    assert rep_sg.is_valid, f"Expected valid Scene Graph JSON to pass, got: {[str(i) for i in rep_sg.issues]}"

    gz_status = "WITH native `gz sdf -k`" if rep_ok.gz_cli_checked else "standalone Python mode"
    print(f"[PASS] All SDF & Scene Graph validation self-tests succeeded ({gz_status}).")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate Custebo Scene Graph (.json/.gproj) and Gazebo (.sdf/.world) files."
    )
    parser.add_argument("files", nargs="*", help="Paths to .sdf, .world, .json, or .gproj files")
    parser.add_argument("--self-test", action="store_true", help="Run built-in validator self-tests")
    args = parser.parse_args()

    if args.self_test:
        return run_self_test()

    if not args.files:
        parser.print_help()
        return 1

    overall_ok = True
    for file_path in args.files:
        path = Path(file_path)
        if not path.is_file():
            print(f"[ERROR] File not found: {file_path}", file=sys.stderr)
            overall_ok = False
            continue

        content = path.read_text(encoding="utf-8")
        if path.suffix.lower() in (".json", ".gproj"):
            try:
                data = json.loads(content)
                report = validate_scene_graph_dict(data, str(path))
            except json.JSONDecodeError as exc:
                report = ValidationReport(target=str(path))
                report.add_error("JSON_PARSE_ERROR", str(exc), str(path))
        else:
            report = validate_sdf_content(content, str(path))

        status = "VALID" if report.is_valid else "INVALID"
        gz_note = " (verified by `gz sdf -k`)" if report.gz_cli_checked else ""
        print(f"=== {path} -> {status}{gz_note} ===")
        for issue in report.issues:
            print(f"  {issue}")
        if not report.is_valid:
            overall_ok = False

    return 0 if overall_ok else 1


if __name__ == "__main__":
    sys.exit(main())

