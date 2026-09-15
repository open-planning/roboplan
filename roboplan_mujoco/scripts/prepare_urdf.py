#!/usr/bin/env python3

""" Convert the registered (Xacro/URDF) RoboPlan example models to MuJoCo URDFs.
This script does following:
1. expand xacro files 
2. convert meshes to mujoco compatible formats (.obj)
3. assign material colors to the meshes
4. ensure unique geometry names
5. remove planar joint limits to avoid Mujoco applying a scalar limit to only one planar translation axis or applying the same limit to all axes.

The generated URDFs and converted mesh assets are stored under 'roboplan_example_models/models/mujoco'.
"""

from __future__ import annotations

import argparse
import copy
from dataclasses import dataclass
import hashlib
import os
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from trimesh.exchange.obj import export_obj
import xacro

MUJOCO_MESH_EXTENSIONS = {".msh", ".obj", ".stl"}

@dataclass(frozen=True)
class RobotModel:
    source: Path
    output_name: str

REGISTERED_MODELS = {
    "dual_franka": RobotModel(Path("franka_robot_model/dual_fr3.urdf"), "dual_fr3.urdf"),
    "franka": RobotModel(Path("franka_robot_model/fr3.urdf"), "fr3.urdf"),
    "kinova": RobotModel(Path("kinova_robot_model/kinova_robotiq.urdf"), "kinova_robotiq.urdf"),
    "reachback": RobotModel(Path("reachback_robot_model/reachback.urdf"), "reachback.urdf"),
    "so101": RobotModel(Path("so101_robot_model/so101.urdf"), "so101.urdf"),
    "stretch": RobotModel(Path("stretch4_robot_model/stretch4_sg4.urdf"), "stretch4_sg4.urdf"),
    "ur5": RobotModel(Path("ur_robot_model/ur5_gripper.urdf"), "ur5_gripper.urdf"),
}

def _default_models_directory() -> Path:
    source_models = (Path(__file__).resolve().parents[2] / "roboplan_example_models" / "models")
    if source_models.is_dir():
        return source_models

    try:
        from roboplan.example_models import get_package_models_dir
    except ImportError as error:
        raise FileNotFoundError("Could not locate roboplan_example_models; pass --models-directory") from error
    return Path(get_package_models_dir())

def _parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Prepare the registered RoboPlan example models for Mujoco. "
            "With no model names specified, every registered model is prepared."
        )
    )
    parser.add_argument(
        "models",
        nargs="*",
        metavar="MODEL",
        help=(
            "Models to prepare (default: all). Available: "
            + ", ".join(REGISTERED_MODELS)
        ),
    )
    parser.add_argument(
        "--models-directory",
        type=Path,
        help=(
            "Override the roboplan_example_models/models directory used for input "
            "and generated output"
        ),
    )
    return parser.parse_args()

def _resolve_mesh_path(filename: str, input_directory: Path, package_roots: list[Path]) -> Path:
    package_prefix = "package://"
    if filename.startswith(package_prefix):
        package_reference = filename[len(package_prefix) :]
        package_name, separator, relative_path = package_reference.partition("/")
        if not separator:
            raise ValueError(f"Invalid package URI: {filename}")
        for root in package_roots:
            candidates = [root / package_name / relative_path]
            if root.name == package_name:
                candidates.append(root / relative_path)
            for candidate in candidates:
                if candidate.is_file():
                    return candidate.resolve()
        raise FileNotFoundError(f"Could not resolve {filename}; searched package roots: {package_roots}")

    if filename.startswith("file://"):
        path = Path(filename[len("file://") :])
    else:
        path = Path(filename)
    if not path.is_absolute():
        path = input_directory / path
    if not path.is_file():
        raise FileNotFoundError(f"Mesh file does not exist: {path}")
    return path.resolve()

def _material_color(mesh: trimesh.Trimesh) -> np.ndarray | None:
    material = getattr(mesh.visual, "material", None)
    color = getattr(material, "baseColorFactor", None)
    if color is None:
        return None
    values = np.asarray(color, dtype=float)
    if values.shape != (4,):
        return None
    if np.max(values) > 1.0:
        values /= 255.0
    return np.clip(values, 0.0, 1.0)

def _mesh_digest(source: Path) -> str:
    return hashlib.sha256(source.read_bytes()).hexdigest()[:12]

def _convert_mesh(source: Path, asset_directory: Path) -> list[tuple[Path, np.ndarray | None]]:
    digest = _mesh_digest(source)
    scene = trimesh.load_scene(source)
    converted: list[tuple[Path, np.ndarray | None]] = []
    for index, node_name in enumerate(sorted(scene.graph.nodes_geometry)):
        transform, geometry_name = scene.graph[node_name]
        mesh = scene.geometry[geometry_name].copy()
        mesh.apply_transform(transform)
        output = asset_directory / f"{source.stem}_{digest}_{index}.obj"
        output.write_text(
            export_obj(mesh, include_color=False, include_texture=False),
            encoding="utf-8",
        )
        converted.append((output.resolve(), _material_color(mesh)))
    if not converted:
        raise ValueError(f"Mesh contains no geometry: {source}")
    return converted

def _asset_reference(asset: Path, output_directory: Path) -> str:
    return Path(os.path.relpath(asset, output_directory)).as_posix()

def _set_material(visual: ET.Element, name: str, color: np.ndarray | None) -> None:
    if color is None:
        return
    material = visual.find("material")
    if material is None:
        material = ET.SubElement(visual, "material")
    material.set("name", name)
    for child in list(material):
        material.remove(child)
    color_element = ET.SubElement(material, "color")
    color_element.set("rgba", " ".join(f"{component:.9g}" for component in color))

def _ensure_unique_geometry_names(robot: ET.Element) -> None:
    used_names: set[str] = set()
    for link in robot.findall("link"):
        link_name = link.get("name", "link")
        for element_type in ("visual", "collision"):
            for index, element in enumerate(link.findall(element_type)):
                base_name = element.get("name", f"{link_name}_{element_type}_{index}")
                name = base_name
                suffix = 1
                while name in used_names:
                    name = f"{base_name}_{suffix}"
                    suffix += 1
                element.set("name", name)
                used_names.add(name)

# NOTE: this is a temporary fix, until we figure out how to apply the scalar limits to all axes independently
def _remove_planar_joint_limits(robot: ET.Element) -> None:
    """Avoid Mujoco applying a scalar limit to only one planar translation axis."""
    for joint in robot.findall("joint"):
        if joint.get("type") != "planar":
            continue
        limit = joint.find("limit")
        if limit is not None:
            joint.remove(limit)

def _prepare_mesh_elements(robot: ET.Element,element_type: str,input_directory: Path,package_roots: list[Path],asset_directory: Path,output_directory: Path,conversions: dict[Path, list[tuple[Path, np.ndarray | None]]],material_index: int) -> int:
    for link in robot.findall("link"):
        original_elements = list(link.findall(element_type))
        for element in original_elements:
            mesh = element.find("./geometry/mesh")
            if mesh is None or "filename" not in mesh.attrib:
                continue
            source = _resolve_mesh_path(mesh.attrib["filename"], input_directory, package_roots)
            if source.suffix.lower() in MUJOCO_MESH_EXTENSIONS:
                mesh.set("filename", _asset_reference(source, output_directory))
                continue
            if source not in conversions:
                conversions[source] = _convert_mesh(source, asset_directory)

            insertion_index = list(link).index(element)
            link.remove(element)
            for part_index, (part_path, color) in enumerate(conversions[source]):
                converted_element = copy.deepcopy(element)
                converted_mesh = converted_element.find("./geometry/mesh")
                assert converted_mesh is not None
                converted_mesh.set("filename", _asset_reference(part_path, output_directory))
                base_name = element.get("name", f"{link.get('name', 'link')}_{element_type}")
                converted_element.set("name", f"{base_name}_{part_index}")
                if element_type == "visual":
                    _set_material(converted_element,f"roboplan_mujoco_material_{material_index}",color)
                    material_index += 1
                link.insert(insertion_index + part_index, converted_element)
    return material_index

def _prepare_meshes(robot: ET.Element, input_directory: Path, package_roots: list[Path], asset_directory: Path, output_directory: Path) -> None:
    conversions: dict[Path, list[tuple[Path, np.ndarray | None]]] = {}
    material_index = _prepare_mesh_elements(robot,"visual",input_directory,package_roots,asset_directory,output_directory,conversions,0)
    _prepare_mesh_elements(robot,"collision",input_directory,package_roots,asset_directory,output_directory,conversions,material_index)

def prepare_urdf(input_path: Path, output_path: Path, package_roots: list[Path]) -> None:
    input_path = input_path.resolve()
    output_path = output_path.resolve()
    if not input_path.is_file():
        raise FileNotFoundError(f"Input description does not exist: {input_path}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    asset_directory = output_path.parent / f"{output_path.stem}_assets"
    if asset_directory.exists():
        shutil.rmtree(asset_directory)
    asset_directory.mkdir(parents=True, exist_ok=True)

    document = xacro.process_file(str(input_path))
    robot = ET.fromstring(document.toxml())
    if robot.tag != "robot":
        raise ValueError(f"Expected a <robot> root element, got <{robot.tag}>")
    _prepare_meshes(robot, input_path.parent, [root.resolve() for root in package_roots], asset_directory,output_path.parent)
    _remove_planar_joint_limits(robot)
    _ensure_unique_geometry_names(robot)
    ET.indent(robot, space="  ")
    output_path.write_text(
        '<?xml version="1.0"?>\n' + ET.tostring(robot, encoding="unicode") + "\n",
        encoding="utf-8",
    )

def prepare_registered_model(model_name: str, models_directory: Path) -> Path:
    model = REGISTERED_MODELS[model_name]
    output_path = models_directory / "mujoco" / model_name / model.output_name
    prepare_urdf(models_directory / model.source, output_path, [models_directory.parents[1]])
    return output_path

def main() -> int:
    arguments = _parse_arguments()
    try:
        models_directory = (arguments.models_directory.resolve() if arguments.models_directory else _default_models_directory())
        model_names = arguments.models or list(REGISTERED_MODELS)
        unknown_models = sorted(set(model_names) - REGISTERED_MODELS.keys())
        if unknown_models:
            raise ValueError(
                "Unknown model(s): "
                + ", ".join(unknown_models)
                + ". Available models: "
                + ", ".join(REGISTERED_MODELS)
            )
        for model_name in model_names:
            output_path = prepare_registered_model(model_name, models_directory)
            print(f"Prepared {model_name}: {output_path.resolve()}")
    except Exception as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0

if __name__ == "__main__":
    raise SystemExit(main())