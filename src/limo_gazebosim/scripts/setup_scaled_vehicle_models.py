#!/usr/bin/env python3

import argparse
import math
import random
import shutil
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path


WORLD_NAME = "parking_lot_scaled_vehicles.world"
LEGACY_MANAGED_INCLUDE_NAMES = (
    "scaled_prius_hybrid_instance",
    "scaled_golf_cart_instance",
    "scaled_delivery_truck_instance",
    "scaled_passenger_car_instance",
    "scaled_suv_instance",
)
MANAGED_INCLUDE_NAME_PREFIXES = (
    "parked_",
)
MANAGED_INCLUDE_URIS = (
    "model://scaled_prius_hybrid",
    "model://scaled_golf_cart",
    "model://scaled_delivery_truck",
    "model://scaled_passenger_car",
    "model://scaled_suv",
)
MODEL_SPECS = (
    {
        "requested_name": "prius_hybrid",
        "official_source": "prius_hybrid",
        "source_root": "official",
        "scale_factor": 0.14,
        "static": True,
        "note": "Exact official Gazebo model, scaled to 1.4x the previous 1/10 size.",
    },
    {
        "requested_name": "passenger_car",
        "official_source": "hatchback_red",
        "source_root": "official",
        "scale_factor": 0.14,
        "static": False,
        "mass_kg": 1.2,
        "inertia_box_m": (0.55, 0.24, 0.20),
        "note": "Official Gazebo hatchback used as the additional scaled passenger car, enlarged to 1.4x the previous 1/10 size.",
    },
    {
        "requested_name": "suv",
        "official_source": "suv",
        "source_root": "local",
        "scale_factor": 0.1,
        "static": True,
        "note": "Local Gazebo SUV model scaled for the parking scenario.",
    },
)
SLOT_WIDTH_M = 0.340909
COLUMN_COUNT = 11
ROW_LAYOUT = {
    1: {"y": 0.75, "yaw": -1.5708},
    2: {"y": -0.75, "yaw": 1.5708},
    3: {"y": -1.65, "yaw": 1.5708},
}
LIMO2_RESERVED_SLOT = (2, 6)
REMOVED_MOVING_PASSENGER_INCLUDE_NAME = "moving_passenger_car"
STATIC_PARKED_MODELS = (
    "scaled_prius_hybrid",
    "scaled_suv",
)
STATIC_VEHICLES_PER_MODEL = 16
DEFAULT_VEHICLE_SEED = 20260505
VEHICLE_Z_M = 0.02
COLLISION_THRESHOLD_M = 0.236
PLANNED_LIMO_SPAWN_X_M = 0.0215
PLANNED_LIMO_SPAWN_Y_M = -0.1030
PLANNED_LIMO2_SPAWN_X_M = 0.0
PLANNED_LIMO2_SPAWN_Y_M = -0.75
MANAGED_MODEL_DIRS = (
    "golf_cart",
    "delivery_truck",
    "scaled_golf_cart",
    "scaled_delivery_truck",
    "prius_hybrid",
    "scaled_prius_hybrid",
    "passenger_car",
    "scaled_passenger_car",
    "suv",
    "scaled_suv",
)


def strip_namespace(tag: str) -> str:
    return tag.split("}", 1)[-1]


def indent_xml(elem: ET.Element, level: int = 0) -> None:
    indent = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = indent + "  "
        for child in elem:
            indent_xml(child, level + 1)
        if not elem[-1].tail or not elem[-1].tail.strip():
            elem[-1].tail = indent
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = indent


def scaled_numbers(text: str, scale: float, pose: bool = False) -> str:
    values = [float(item) for item in text.split()]
    if pose:
        for index in range(min(3, len(values))):
            values[index] *= scale
    else:
        values = [value * scale for value in values]
    return " ".join(format_number(value) for value in values)


def format_number(value: float) -> str:
    text = f"{value:.8f}".rstrip("0").rstrip(".")
    return text if text else "0"


def slot_label(row: int, column: int) -> str:
    return f"R{row}C{column}"


def is_target_slot(row: int, column: int) -> bool:
    return (row, column) == LIMO2_RESERVED_SLOT


@dataclass
class SpawnRecord:
    name: str
    model_name: str
    x: float
    y: float
    yaw: float
    is_static: bool


@dataclass
class SpawnRegistry:
    occupied: dict = field(default_factory=dict)

    def reserve(
        self,
        row: int,
        column: int,
        name: str,
        model_name: str,
        x: float,
        y: float,
        yaw: float,
        is_static: bool,
    ) -> None:
        key = (row, column)
        if is_target_slot(row, column) and is_static:
            raise RuntimeError(
                f"[TARGET_SLOT] {slot_label(row, column)} is reserved for "
                f"limo2; static model '{name}' is not allowed."
            )
        if key in self.occupied:
            existing = self.occupied[key]
            raise RuntimeError(
                f"[DUPLICATE] {slot_label(row, column)} already occupied by "
                f"'{existing.name}' at ({existing.x:.3f}, {existing.y:.3f})."
            )
        self.occupied[key] = SpawnRecord(
            name=name,
            model_name=model_name,
            x=x,
            y=y,
            yaw=yaw,
            is_static=is_static,
        )

    def assert_clear(
        self,
        x: float,
        y: float,
        name: str,
        threshold: float = COLLISION_THRESHOLD_M,
    ) -> None:
        for key, record in self.occupied.items():
            distance = math.hypot(x - record.x, y - record.y)
            if distance < threshold:
                raise RuntimeError(
                    f"[COLLISION] '{name}' at ({x:.3f}, {y:.3f}) vs "
                    f"{slot_label(*key)} '{record.name}' at "
                    f"({record.x:.3f}, {record.y:.3f}); "
                    f"d={distance:.3f} < {threshold:.3f}"
                )


def slot_center_x(column: int) -> float:
    return (column - 6) * SLOT_WIDTH_M


def build_world_layout(vehicle_seed: int) -> tuple:
    static_slots = []
    target_skip_seen = False
    for row in sorted(ROW_LAYOUT):
        for column in range(1, COLUMN_COUNT + 1):
            if is_target_slot(row, column):
                print(
                    f"[SKIP] target slot {slot_label(row, column)} reserved for "
                    "limo2"
                )
                target_skip_seen = True
                continue
            static_slots.append((row, column))
    if not target_skip_seen:
        raise RuntimeError(
            f"[TARGET_SLOT] {slot_label(*LIMO2_RESERVED_SLOT)} skip was not applied."
        )

    static_models = [
        model_name
        for model_name in STATIC_PARKED_MODELS
        for _ in range(STATIC_VEHICLES_PER_MODEL)
    ]
    if len(static_slots) != len(static_models):
        raise RuntimeError(
            f"Static slot/model count mismatch: {len(static_slots)} slots, "
            f"{len(static_models)} models."
        )

    rng = random.Random(vehicle_seed)
    rng.shuffle(static_models)
    static_by_slot = dict(zip(static_slots, static_models))
    if LIMO2_RESERVED_SLOT in static_by_slot:
        raise RuntimeError(
            f"[TARGET_SLOT] {slot_label(*LIMO2_RESERVED_SLOT)} was assigned a static vehicle."
        )

    registry = SpawnRegistry()
    layout = []
    for row in sorted(ROW_LAYOUT):
        row_data = ROW_LAYOUT[row]
        for column in range(1, COLUMN_COUNT + 1):
            x = slot_center_x(column)
            y = row_data["y"]
            if is_target_slot(row, column):
                continue

            model_name = static_by_slot[(row, column)]
            yaw = row_data["yaw"]
            if model_name == "scaled_prius_hybrid":
                yaw += 1.5708
            short_model_name = model_name.removeprefix("scaled_")
            include_name = f"parked_{short_model_name}_r{row}_c{column:02d}"
            pose = (x, y, VEHICLE_Z_M, 0.0, 0.0, yaw)
            registry.assert_clear(x, y, include_name)
            registry.reserve(
                row=row,
                column=column,
                name=include_name,
                model_name=model_name,
                x=x,
                y=y,
                yaw=yaw,
                is_static=True,
            )
            layout.append((include_name, model_name, True, pose))

    registry.assert_clear(
        PLANNED_LIMO_SPAWN_X_M,
        PLANNED_LIMO_SPAWN_Y_M,
        "limo1",
    )
    registry.assert_clear(
        PLANNED_LIMO2_SPAWN_X_M,
        PLANNED_LIMO2_SPAWN_Y_M,
        "limo2",
    )
    return tuple(layout)


def is_managed_include(include_elem: ET.Element) -> bool:
    include_name = None
    include_uri = None
    for child in include_elem:
        if strip_namespace(child.tag) == "name":
            include_name = (child.text or "").strip()
        elif strip_namespace(child.tag) == "uri":
            include_uri = (child.text or "").strip()

    if include_name == REMOVED_MOVING_PASSENGER_INCLUDE_NAME:
        return True
    if include_name in LEGACY_MANAGED_INCLUDE_NAMES:
        return True
    if include_name and include_name.startswith(MANAGED_INCLUDE_NAME_PREFIXES):
        return True
    return include_uri in MANAGED_INCLUDE_URIS


def ensure_model_static(model_elem: ET.Element, is_static: bool) -> None:
    static_elem = None
    for child in model_elem:
        if strip_namespace(child.tag) == "static":
            static_elem = child
            break
    if static_elem is None:
        static_elem = ET.Element("static")
        static_elem.text = "true" if is_static else "false"
        insert_index = 0
        for index, child in enumerate(list(model_elem)):
            if strip_namespace(child.tag) == "pose":
                insert_index = index + 1
                break
        model_elem.insert(insert_index, static_elem)
    else:
        static_elem.text = "true" if is_static else "false"


def box_inertia(mass: float, size_x: float, size_y: float, size_z: float) -> dict:
    return {
        "ixx": mass * (size_y * size_y + size_z * size_z) / 12.0,
        "iyy": mass * (size_x * size_x + size_z * size_z) / 12.0,
        "izz": mass * (size_x * size_x + size_y * size_y) / 12.0,
        "ixy": 0.0,
        "ixz": 0.0,
        "iyz": 0.0,
    }


def find_first_child(parent: ET.Element, tag_name: str) -> ET.Element:
    for child in parent:
        if strip_namespace(child.tag) == tag_name:
            return child
    child = ET.Element(tag_name)
    parent.append(child)
    return child


def ensure_link_inertial(link_elem: ET.Element) -> ET.Element:
    for child in link_elem:
        if strip_namespace(child.tag) == "inertial":
            return child

    inertial_elem = ET.Element("inertial")
    insert_before_tags = {
        "collision",
        "visual",
        "sensor",
        "projector",
        "audio_sink",
        "audio_source",
        "battery",
        "light",
    }
    insert_index = len(link_elem)
    for index, child in enumerate(list(link_elem)):
        if strip_namespace(child.tag) in insert_before_tags:
            insert_index = index
            break
    link_elem.insert(insert_index, inertial_elem)
    return inertial_elem


def ensure_dynamic_link_inertial(
    model_elem: ET.Element,
    mass_kg: float,
    inertia_box_m: tuple,
) -> None:
    link_elem = next(
        (elem for elem in model_elem if strip_namespace(elem.tag) == "link"),
        None,
    )
    if link_elem is None:
        raise RuntimeError(
            f"No <link> found in dynamic model '{model_elem.get('name', 'unknown')}'"
        )

    inertial_elem = ensure_link_inertial(link_elem)
    mass_elem = find_first_child(inertial_elem, "mass")
    mass_elem.text = format_number(mass_kg)

    pose_elem = find_first_child(inertial_elem, "pose")
    pose_elem.text = "0 0 0 0 0 0"

    inertia_elem = find_first_child(inertial_elem, "inertia")
    inertia_values = box_inertia(mass_kg, *inertia_box_m)
    for key, value in inertia_values.items():
        item_elem = find_first_child(inertia_elem, key)
        item_elem.text = format_number(value)


def ensure_mesh_scale(mesh_elem: ET.Element, scale: float) -> None:
    scale_elem = None
    for child in mesh_elem:
        if strip_namespace(child.tag) == "scale":
            scale_elem = child
            break
    if scale_elem is None:
        scale_elem = ET.Element("scale")
        scale_elem.text = f"{scale} {scale} {scale}"
        mesh_elem.append(scale_elem)
    else:
        scale_elem.text = scaled_numbers(scale_elem.text or "1 1 1", scale, pose=False)


def patch_model_config(
    config_path: Path,
    model_name: str,
    note: str,
    is_static: bool = True,
) -> None:
    tree = ET.parse(config_path)
    root = tree.getroot()
    name_elem = root.find("name")
    if name_elem is None:
        name_elem = ET.SubElement(root, "name")
    name_elem.text = model_name

    description_elem = root.find("description")
    model_behavior = "static" if is_static else "dynamic"
    description_text = (
        f"{model_name}. Auto-generated 1/10 {model_behavior} vehicle model. Source note: {note}"
    )
    if description_elem is None:
        description_elem = ET.SubElement(root, "description")
    description_elem.text = description_text

    indent_xml(root)
    tree.write(config_path, encoding="utf-8", xml_declaration=True)


def scale_sdf_model(
    sdf_path: Path,
    model_name: str,
    model_uri_name: str,
    scale: float,
    is_static: bool,
    mass_kg: float = None,
    inertia_box_m: tuple = None,
) -> None:
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    model_elem = next((elem for elem in root.iter() if strip_namespace(elem.tag) == "model"), None)
    if model_elem is None:
        raise RuntimeError(f"No <model> found in {sdf_path}")

    model_elem.set("name", model_name)
    ensure_model_static(model_elem, is_static)
    if not is_static:
        if mass_kg is None or inertia_box_m is None:
            raise RuntimeError(f"Dynamic model '{model_name}' is missing mass or inertia settings.")
        ensure_dynamic_link_inertial(model_elem, mass_kg, inertia_box_m)

    for elem in root.iter():
        tag = strip_namespace(elem.tag)
        if tag == "pose" and elem.text:
            elem.text = scaled_numbers(elem.text, scale, pose=True)
        elif tag == "size" and elem.text:
            elem.text = scaled_numbers(elem.text, scale, pose=False)
        elif tag in {"radius", "length"} and elem.text:
            elem.text = format_number(float(elem.text) * scale)
        elif tag == "mesh":
            ensure_mesh_scale(elem, scale)
        elif tag == "uri" and elem.text and elem.text.strip().startswith("model://"):
            uri_text = elem.text.strip()
            uri_parts = uri_text.split("/", 3)
            if len(uri_parts) >= 3:
                uri_parts[2] = model_uri_name
                elem.text = "/".join(uri_parts)

    indent_xml(root)
    tree.write(sdf_path, encoding="utf-8", xml_declaration=True)


def alias_unscaled_model(
    source_dir: Path,
    dest_dir: Path,
    model_name: str,
    source_name: str,
    note: str,
) -> None:
    if dest_dir.exists():
        shutil.rmtree(dest_dir)
    shutil.copytree(source_dir, dest_dir)

    config_path = dest_dir / "model.config"
    sdf_path = dest_dir / "model.sdf"
    if config_path.exists():
        patch_model_config(config_path, model_name, note)
    if sdf_path.exists():
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        model_elem = next(
            (elem for elem in root.iter() if strip_namespace(elem.tag) == "model"),
            None,
        )
        if model_elem is not None:
            model_elem.set("name", model_name)
        for elem in root.iter():
            if strip_namespace(elem.tag) == "uri" and elem.text:
                elem.text = elem.text.replace(f"model://{source_name}", f"model://{model_name}")
        indent_xml(root)
        tree.write(sdf_path, encoding="utf-8", xml_declaration=True)


def create_scaled_model(
    original_dir: Path,
    scaled_dir: Path,
    scaled_name: str,
    note: str,
    scale_factor: float,
    is_static: bool,
    mass_kg: float = None,
    inertia_box_m: tuple = None,
) -> None:
    if scaled_dir.exists():
        shutil.rmtree(scaled_dir)
    shutil.copytree(original_dir, scaled_dir)

    config_path = scaled_dir / "model.config"
    sdf_path = scaled_dir / "model.sdf"
    if config_path.exists():
        patch_model_config(config_path, scaled_name, note, is_static=is_static)
    if sdf_path.exists():
        scale_sdf_model(
            sdf_path,
            scaled_name,
            scaled_name,
            scale_factor,
            is_static=is_static,
            mass_kg=mass_kg,
            inertia_box_m=inertia_box_m,
        )


def update_world(world_path: Path, vehicle_seed: int) -> None:
    tree = ET.parse(world_path)
    root = tree.getroot()
    world_elem = next((elem for elem in root.iter() if strip_namespace(elem.tag) == "world"), None)
    if world_elem is None:
        raise RuntimeError(f"No <world> found in {world_path}")

    for child in list(world_elem):
        tag = strip_namespace(child.tag)
        if tag == "model" and child.get("name") == "parked_suv":
            world_elem.remove(child)
        elif tag == "include" and is_managed_include(child):
            world_elem.remove(child)

    world_elem.append(
        ET.Comment(
            " Vehicle grid: 11 columns x 3 rows, x(C) = (C - 6) * 0.340909. "
            f"Seed={vehicle_seed}. "
        )
    )
    for include_name, model_name, is_static, pose in build_world_layout(vehicle_seed):
        include_elem = ET.Element("include")
        uri_elem = ET.SubElement(include_elem, "uri")
        uri_elem.text = f"model://{model_name}"
        name_elem = ET.SubElement(include_elem, "name")
        name_elem.text = include_name
        static_elem = ET.SubElement(include_elem, "static")
        static_elem.text = "true" if is_static else "false"
        pose_elem = ET.SubElement(include_elem, "pose")
        pose_elem.text = " ".join(format_number(value) for value in pose)
        world_elem.append(include_elem)

    indent_xml(root)
    tree.write(world_path, encoding="utf-8", xml_declaration=True)


def cleanup_managed_model_dirs(models_root: Path) -> None:
    current_model_dirs = {
        spec["requested_name"] for spec in MODEL_SPECS
    } | {
        f"scaled_{spec['requested_name']}" for spec in MODEL_SPECS
    }
    for directory_name in MANAGED_MODEL_DIRS:
        if directory_name in current_model_dirs:
            continue
        target = models_root / directory_name
        if target.exists():
            shutil.rmtree(target)


def resolve_source_dir(source_root: Path, requested_name: str, official_source: str, models_root: Path) -> Path:
    source_dir = source_root / official_source
    if source_dir.exists():
        return source_dir

    local_copy = models_root / requested_name
    if local_copy.exists():
        return local_copy

    raise FileNotFoundError(f"Missing source model '{official_source}' in {source_root}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Prepare scaled local Gazebo vehicle models and place them in the scaled parking world."
    )
    parser.add_argument(
        "--official-source-root",
        type=Path,
        default=Path("/tmp/gazebo_models_osrf"),
        help="Path to the cloned official Gazebo models repository.",
    )
    parser.add_argument(
        "--local-source-root",
        type=Path,
        default=Path("/home/leesumyeong/.gazebo/models"),
        help="Path to the local Gazebo model cache.",
    )
    parser.add_argument(
        "--models-root",
        type=Path,
        default=Path("/home/leesumyeong/parking_ws/src/limo_gazebosim/models"),
        help="Target local models directory.",
    )
    parser.add_argument(
        "--world",
        type=Path,
        default=Path("/home/leesumyeong/parking_ws/src/limo_gazebosim/worlds") / WORLD_NAME,
        help="Target scaled parking world file.",
    )
    parser.add_argument(
        "--vehicle-seed",
        type=int,
        default=DEFAULT_VEHICLE_SEED,
        help="Seed used to shuffle the 16 Prius / 16 SUV static vehicle assignments.",
    )
    args = parser.parse_args()

    args.models_root.mkdir(parents=True, exist_ok=True)
    cleanup_managed_model_dirs(args.models_root)

    for spec in MODEL_SPECS:
        source_root = (
            args.official_source_root
            if spec["source_root"] == "official"
            else args.local_source_root
        )
        original_dir = args.models_root / spec["requested_name"]
        scaled_dir = args.models_root / f"scaled_{spec['requested_name']}"
        source_dir = resolve_source_dir(
            source_root=source_root,
            requested_name=spec["requested_name"],
            official_source=spec["official_source"],
            models_root=args.models_root,
        )

        if source_dir != original_dir:
            alias_unscaled_model(
                source_dir=source_dir,
                dest_dir=original_dir,
                model_name=spec["requested_name"],
                source_name=spec["official_source"],
                note=spec["note"],
            )
        create_scaled_model(
            original_dir=original_dir,
            scaled_dir=scaled_dir,
            scaled_name=f"scaled_{spec['requested_name']}",
            note=spec["note"],
            scale_factor=spec["scale_factor"],
            is_static=spec["static"],
            mass_kg=spec.get("mass_kg"),
            inertia_box_m=spec.get("inertia_box_m"),
        )

    update_world(args.world, args.vehicle_seed)
    print(f"Prepared local models in: {args.models_root}")
    print(f"Updated world: {args.world}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
