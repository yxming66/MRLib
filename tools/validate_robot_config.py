"""Validate the v2 robot configuration contract; never generate or write files."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

try:
    import yaml
    from jsonschema import Draft202012Validator
except ImportError:
    raise SystemExit("Install config dependencies: python -m pip install -r tools/requirements-config.txt")

ROOT = Path(__file__).resolve().parents[1]


class UniqueLoader(yaml.SafeLoader):
    """Reject duplicate keys rather than silently keeping the last value."""


def unique_mapping(loader, node):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node)
        if not isinstance(key, str):
            raise ValueError(f"mapping keys must be strings at line {key_node.start_mark.line + 1}")
        if key in result:
            raise ValueError(f"duplicate key {key!r} at line {key_node.start_mark.line + 1}")
        result[key] = loader.construct_object(value_node)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)


def validate(config: dict) -> None:
    # Fail on NaN/Inf and recursive/unsupported objects before schema traversal.
    try:
        json.dumps(config, allow_nan=False)
    except (TypeError, ValueError, RecursionError) as error:
        raise ValueError(f"configuration must contain finite JSON-compatible values: {error}") from error
    schema = json.loads((ROOT / "docs/robot.schema.json").read_text(encoding="utf-8"))
    Draft202012Validator.check_schema(schema)
    errors = sorted(Draft202012Validator(schema).iter_errors(config), key=lambda e: str(e.absolute_path))
    if errors:
        error = errors[0]
        path = ".".join(str(part) for part in error.absolute_path) or "<root>"
        raise ValueError(f"{path}: {error.message}")

    bindings = config["board"]["bindings"]
    for category, entries in bindings.items():
        if len(entries.values()) != len(set(entries.values())):
            raise ValueError(f"board.bindings.{category}: one physical endpoint has multiple aliases")
    motors = config["devices"]["motors"]
    wheels = config["devices"]["wheels"]
    joints = config["devices"]["joints"]
    modules = config["modules"]
    links = config["communications"]

    def require(name, registry, location):
        if name not in registry:
            raise ValueError(f"{location}: unknown reference {name!r}")

    occupied = {}

    def claim(bus, endpoint, owner):
        key = (bus, endpoint)
        if key in occupied and occupied[key] != owner:
            raise ValueError(f"CAN endpoint conflict on {bus}: {endpoint} ({occupied[key]} / {owner})")
        occupied[key] = owner

    rm_groups = set()
    rm_slots = set()
    for name, motor in motors.items():
        bus = motor["bus"]
        require(bus, bindings["can"], f"devices.motors.{name}.bus")
        if motor["kind"] == "rm":
            feedback = motor["feedback_id"]
            low, high = (0x205, 0x20b) if motor["model"] == "M6020" else (0x201, 0x208)
            if not low <= feedback <= high:
                raise ValueError(f"{name}: invalid RM feedback ID for {motor['model']}")
            group = (0x1ff if feedback <= 0x208 else 0x2ff) if low == 0x205 else (
                0x200 if feedback <= 0x204 else 0x1ff)
            slot = (feedback - low) % 4
            if (bus, group, slot) in rm_slots:
                raise ValueError(f"{name}: duplicate RM command slot")
            rm_slots.add((bus, group, slot))
            claim(bus, feedback, name)
            rm_groups.add((bus, group))
        elif motor["kind"] == "dm":
            for endpoint in (motor["master_id"], motor["can_id"],
                             motor["can_id"] + 0x100, motor["can_id"] + 0x200):
                claim(bus, endpoint, name)
        else:
            if motor["motor_id"] == motor["host_id"]:
                raise ValueError(f"{name}: LZ motor_id and host_id must differ")
            claim(bus, ("lz_target", motor["motor_id"]), name)
    for bus, group in rm_groups:
        claim(bus, group, "RM command group")

    motor_users = {}
    for category, entries in (("wheels", wheels), ("joints", joints)):
        for name, device in entries.items():
            motor = device["motor"]
            require(motor, motors, f"devices.{category}.{name}.motor")
            if motor in motor_users:
                raise ValueError(f"{motor}: motor assigned to both {motor_users[motor]} and {name}")
            motor_users[motor] = name
            if category == "joints":
                if motors[motor]["kind"] not in ("dm", "lz") or motors[motor]["model"] == "H3510":
                    raise ValueError(f"{name}: initial arm contract requires DM/LZ MIT support")
                limits = device["limits"]
                if limits["min_rad"] >= limits["max_rad"]:
                    raise ValueError(f"{name}: min_rad must be below max_rad")

    device_users = set()
    for name, module in modules.items():
        if module["type"] == "chassis":
            references = module["wheels"]
            category, registry = "wheel", wheels
        else:
            references = [entry["device"] for entry in module["joints"]]
            category, registry = "joint", joints
            for entry in module["joints"]:
                norm = math.hypot(*entry["axis_in_joint_frame"])
                if not math.isclose(norm, 1.0, rel_tol=1e-5, abs_tol=1e-5):
                    raise ValueError(f"{name}: joint axis must be a unit vector")
        for device in references:
            require(device, registry, f"modules.{name}")
            key = (category, device)
            if key in device_users:
                raise ValueError(f"{name}: {category} {device} has more than one module owner")
            device_users.add(key)

    ports = set()
    for name, link in links.items():
        require(link["port"], bindings["uart"], f"communications.{name}.port")
        if link["port"] in ports:
            raise ValueError(f"{name}: UART already used by another communication link")
        ports.add(link["port"])

    module_owners, link_owners = set(), set()
    for name, task in config["tasks"].items():
        for field, registry, owners in (("owns_modules", modules, module_owners),
                                         ("services", links, link_owners)):
            for reference in task[field]:
                require(reference, registry, f"tasks.{name}.{field}")
                if reference in owners:
                    raise ValueError(f"{reference}: more than one task owner")
                owners.add(reference)
    if module_owners != set(modules) or link_owners != set(links):
        raise ValueError("every module and communication link must have exactly one task owner")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path)
    args = parser.parse_args()
    try:
        config = yaml.load(args.config.read_text(encoding="utf-8"), Loader=UniqueLoader)
        validate(config)
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f"Invalid robot config: {error}", file=sys.stderr)
        return 1
    print(f"Valid v2 robot config: {args.config} (validation only; no code generated)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
