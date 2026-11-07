"""Versioned robot-config validation and deterministic assembly generation."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import tempfile

import yaml

from tools.validate_robot_config import UniqueLoader, validate

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ".mrctl-manifest.json"
GENERATED_HEADER = "robot_config.hpp"

DM_ALIASES = {
    "J10010": "DmJ10010Motor", "J10010L": "DmJ10010LMotor",
    "J10422P": "DmJ10422PMotor", "J3507": "DmJ3507Motor",
    "J4310": "DmJ4310Motor", "J4310P": "DmJ4310PMotor",
    "J4340": "DmJ4340Motor", "J4340P": "DmJ4340PMotor",
    "J6006": "DmJ6006Motor", "J6248P": "DmJ6248PMotor",
    "J8006": "DmJ8006Motor", "J8009": "DmJ8009Motor",
    "J8009P": "DmJ8009PMotor", "H3510": "DmH3510Motor",
}
LZ_ALIASES = {f"RSO{index}": f"LzRso{index}Motor" for index in range(7)}


def load_config(path: Path) -> dict:
    try:
        config = yaml.load(path.read_text(encoding="utf-8"), Loader=UniqueLoader)
    except yaml.YAMLError as error:
        raise ValueError(f"invalid YAML: {error}") from error
    validate(config)
    return config


def _f(value: float) -> str:
    text = format(float(value), ".9g")
    if "." not in text and "e" not in text:
        text += ".0"
    return text + "f"


def plan_chassis(config: dict, module_name: str) -> dict:
    module = config["modules"].get(module_name)
    if not module or module["type"] != "chassis":
        raise ValueError(f"selected module {module_name!r} must be a chassis")
    wheels = config["devices"]["wheels"]
    motors = config["devices"]["motors"]
    selected = [(name, wheels[name], motors[wheels[name]["motor"]]) for name in module["wheels"]]
    if any(motor["kind"] != "rm" for _, _, motor in selected):
        raise ValueError("initial chassis generator supports RM wheel motors only")
    buses = {motor["bus"] for _, _, motor in selected}
    if len(buses) != 1:
        raise ValueError("initial chassis generator requires all four motors on one CAN port")
    bus = next(iter(buses))
    # Generated declarations must fit the fixed embedded scalar representation.
    check_float(module)
    for _, wheel, motor in selected:
        check_float(wheel)
        check_float(motor)
    owner = next(task for task in config["tasks"].values() if module_name in task["owns_modules"])
    period = owner["period_us"]
    if period > 100000:
        raise ValueError("chassis control period must be <= 100000 us (module dt limit)")
    return {"module": module, "wheels": selected, "bus": bus,
            "binding": config["board"]["bindings"]["can"][bus], "period_us": period,
            "targets": ["mr::device_motor", "mr::device_wheel", "mr::module_chassis"]}


def plan_arm(config: dict, module_name: str) -> dict:
    module = config["modules"].get(module_name)
    if not module or module["type"] != "arm":
        raise ValueError(f"selected module {module_name!r} must be an arm")
    if module["enable_gravity_compensation"]:
        raise ValueError("generated arms require gravity compensation disabled until inertial data is configured")
    joints = config["devices"]["joints"]
    motors = config["devices"]["motors"]
    selected = []
    buses = set()
    for index, entry in enumerate(module["joints"]):
        if index > 255:
            raise ValueError("generated JointT supports at most 256 joints")
        name = entry["device"]
        joint = joints[name]
        motor = motors[joint["motor"]]
        if motor["kind"] not in ("dm", "lz") or motor["model"] == "H3510":
            raise ValueError(f"joint {name!r} requires a DM/LZ motor with native MIT support")
        buses.add(motor["bus"])
        selected.append((name, joint, motor, entry))
    check_float(module)
    for _, joint, motor, entry in selected:
        check_float(joint)
        check_float(motor)
        check_float(entry)
    if len(buses) != 1:
        raise ValueError("initial arm generator requires all joints on one CAN port")
    bus = next(iter(buses))
    owner = next(task for task in config["tasks"].values() if module_name in task["owns_modules"])
    period = owner["period_us"]
    if period > 100000:
        raise ValueError("arm control period must be <= 100000 us (controller dt limit)")
    targets = ["mr::device_motor", "mr::device_joint", "mr::module_arm"]
    return {"module": module, "joints": selected, "bus": bus,
            "binding": config["board"]["bindings"]["can"][bus],
            "period_us": period, "targets": targets}


def check_float(value):
    if isinstance(value, (float, int)) and (abs(value) > 3.4028234e38 or
                                    (value != 0 and abs(value) < 1.1754944e-38)):
        raise ValueError("generated numeric values must fit normal float32 (or zero)")
    if isinstance(value, dict):
        for item in value.values():
            check_float(item)
    elif isinstance(value, list):
        for item in value:
            check_float(item)


def _transform_lines(label: str, value: dict) -> list[str]:
    translation = ", ".join(_f(x) for x in value["translation_m"])
    rpy = ", ".join(_f(x) for x in value["rpy_rad"])
    return [f"    const float {label}_translation[] = {{{translation}}};",
            f"    const float {label}_rpy[] = {{{rpy}}};",
            f"    const auto {label}_rotation = mr::robotics::core::RotationFromRpy(",
            f"        mr::robotics::core::Vec3{{{label}_rpy}});",
            f"    const auto {label} = mr::robotics::core::MakeTransform({label}_rotation,",
            f"        mr::robotics::core::Vec3{{{label}_translation}});"]


def render_arm(config: dict, module_name: str) -> str:
    plan = plan_arm(config, module_name)
    module = plan["module"]
    joints = plan["joints"]
    count = len(joints)
    lines = [
        "class ArmAssembly final {", " public:",
        f"  using ArmType = mr::module::arm::Arm<{count}>;",
        f"  static constexpr int kJointCount = {count};",
        f"  static constexpr uint32_t kPeriodUs = {plan['period_us']}u;",
        f"  static constexpr float kDtSeconds = {_f(plan['period_us'] / 1000000)};",
        f"  static constexpr bool kAllowMotorTx = {str(config['runtime']['allow_motor_tx']).lower()};",
        f'  static constexpr const char* kLogicalCan = "{plan["bus"]}";',
        f'  static constexpr const char* kBoardCan = "{plan["binding"]}";',
        "  explicit ArmAssembly(mr::motor::CanPort& port)", "      :"]
    init = []
    for index, (_, _, motor, _) in enumerate(joints):
        init.append(f"        motor_{index}(MotorConfig{index}(port), "
                    f"{{{_f(motor['external_ratio'])}}}),")
    for index in range(count):
        init.append(f"        actuator_{index}(&motor_{index}),")
    for index, (name, joint, _, _) in enumerate(joints):
        name_literal = f'"{name}"'
        init.append(
            f"        joint_{index}({index}, &actuator_{index}, "
            f"{{{_f(joint['limits']['min_rad'])}, {_f(joint['limits']['max_rad'])}, 0.0f, 0.0f}}, "
            f"{_f(joint['zero_offset_rad'])}, {_f(1000000 / plan['period_us'])}, {name_literal})" +
            ("," if index < count - 1 else " {}"))
    lines += init
    lines += ["  ArmAssembly(const ArmAssembly&) = delete;",
              "  ArmAssembly& operator=(const ArmAssembly&) = delete;"]
    # Declaration order keeps borrowed actuator pointers valid until Arm dies.
    for index, (_, _, motor, _) in enumerate(joints):
        alias = (DM_ALIASES if motor["kind"] == "dm" else LZ_ALIASES)[motor["model"]]
        lines.append(f"  mr::motor::{alias} motor_{index};")
    for index in range(count):
        lines.append(f"  mr::MotorActuatorAdapter<decltype(motor_{index})> actuator_{index};")
    for index in range(count):
        lines.append(f"  mr::JointT joint_{index};")
    lines += ["  ArmType arm;", "  int8_t Init() { return arm.Init(Config()); }",
              "  ArmType::Config Config() {", "    ArmType::Config config;",
              "    config.controller.default_dt_s = kDtSeconds;",
              "    config.controller.enable_gravity_compensation = false;"]
    lines += _transform_lines("base", module["base_to_world"])
    lines += _transform_lines("tool", module["tool_to_last_joint"])
    gravity = ", ".join(_f(x) for x in module["gravity_world_m_s2"])
    lines.append(f"    const float gravity[] = {{{gravity}}};")
    lines.append(f"    mr::robotics::arm::JointModel models[{count}]{{}};")
    for index, (_, _, _, entry) in enumerate(joints):
        axis = ", ".join(_f(x) for x in entry["axis_in_joint_frame"])
        limits = joints[index][1]["limits"]
        transform = entry["parent_to_joint"]
        t = ", ".join(_f(x) for x in transform["translation_m"])
        rpy = ", ".join(_f(x) for x in transform["rpy_rad"])
        lines += [f"    const float axis_{index}[] = {{{axis}}};",
                  f"    models[{index}].axis_in_joint_frame = mr::robotics::core::Vec3{{axis_{index}}};",
                  f"    models[{index}].limits.position_enabled = true;",
                  f"    models[{index}].limits.position_min = {_f(limits['min_rad'])};",
                  f"    models[{index}].limits.position_max = {_f(limits['max_rad'])};",
                  f"    models[{index}].limits.max_velocity = {_f(limits['max_velocity_rad_s'])};",
                  f"    models[{index}].limits.max_acceleration = {_f(limits['max_acceleration_rad_s2'])};",
                  f"    const float translation_{index}[] = {{{t}}};",
                  f"    const float rpy_{index}[] = {{{rpy}}};",
                  f"    models[{index}].parent_to_joint = mr::robotics::core::MakeTransform(",
                  f"        mr::robotics::core::RotationFromRpy(mr::robotics::core::Vec3{{rpy_{index}}}),",
                  f"        mr::robotics::core::Vec3{{translation_{index}}});"]
    actuators = ", ".join(f"&joint_{index}" for index in range(count))
    gains = ", ".join(
        "mr::module::arm::ArmOutputAdapter<" + str(count) + ">::Gains{" +
        ", ".join(_f(entry["gains"][key]) for key in
                  ("kp_nm_per_rad", "kd_nm_s_per_rad", "effort_scale")) + "}"
        for _, _, _, entry in joints)
    lines += ["    config.model = {models, base, tool,",
              "        mr::robotics::core::Vec3{gravity}};",
              f"    config.output.actuators = {{{actuators}}};",
              f"    config.output.gains = {{{gains}}};", "    return config;", "  }",
              " private:"]
    for index, (_, _, motor, _) in enumerate(joints):
        kind = "DM" if motor["kind"] == "dm" else "LZ"
        fields = [f"config.port = &port;", f"config.reverse = {str(motor['reverse']).lower()};",
                  f"config.feedback_timeout_ms = {motor['feedback_timeout_ms']}u;"]
        if kind == "DM":
            fields += [f"config.can_id = 0x{motor['can_id']:03x};",
                       f"config.master_id = 0x{motor['master_id']:03x};",
                       f"config.mit_position_range_rad = {_f(motor['mit']['position_range_rad'])};",
                       f"config.mit_velocity_range_rad_s = {_f(motor['mit']['velocity_range_rad_s'])};",
                       f"config.mit_torque_range_nm = {_f(motor['mit']['torque_range_nm'])};"]
        else:
            fields += [f"config.motor_id = {motor['motor_id']}u;",
                       f"config.host_id = {motor['host_id']}u;",
                       "config.mode = mr::motor::LzControlMode::Motion;"]
        lines.append(f"  static mr::motor::MotorInstanceConfig<mr::motor::MotorKind::{kind}> "
                     f"MotorConfig{index}(mr::motor::CanPort& port) {{")
        lines.append(f"    mr::motor::MotorInstanceConfig<mr::motor::MotorKind::{kind}> config;")
        lines += ["    " + field for field in fields]
        lines += ["    return config;", "  }"]
    lines += ["};", ""]
    return "\n".join(lines)


def plan_modules(config: dict, selection: str) -> list[tuple[str, dict]]:
    names = sorted(config["modules"]) if selection == "all" else selection.split(",")
    if not names or len(names) != len(set(names)):
        raise ValueError("select unique module names or --module all")
    plans = []
    buses = set()
    kinds = set()
    for name in names:
        module = config["modules"].get(name)
        if module is None:
            raise ValueError(f"selected module {name!r} is not present in config")
        if module["type"] in kinds:
            raise ValueError("this generator supports at most one chassis and one arm per output")
        kinds.add(module["type"])
        plan = plan_chassis(config, name) if module["type"] == "chassis" else plan_arm(config, name)
        buses.add(plan["bus"])
        plans.append((name, plan))
    if len(buses) != 1:
        raise ValueError("selected modules must use one shared CAN port in this generator version")
    return plans


def render_config(config: dict, selection: str) -> str:
    plans = plan_modules(config, selection)
    chassis_names = [name for name, plan in plans if "wheels" in plan]
    arm_names = [name for name, plan in plans if "joints" in plan]
    include_set = {
        '#include "device/motor/motor.hpp"',
    }
    if chassis_names:
        include_set.update({
            '#include "device/motor/bus/rm_command_bus.hpp"',
            '#include "device/wheel/wheel.hpp"',
            '#include "module/chassis/chassis.hpp"',
        })
    if arm_names:
        include_set.update({
            '#include "device/joint/joint.hpp"',
            '#include "device/joint/motor_actuator_adapter.hpp"',
            '#include "module/arm/arm.hpp"',
            '#include "robotics/core/rotation.hpp"',
            '#include "robotics/core/transform.hpp"',
        })
    body = []
    if chassis_names:
        body.append(render_chassis(config, chassis_names[0]))
    for name in arm_names:
        body.append(render_arm(config, name))
    header = ["// Generated by mrctl. Edit the YAML, not this file.",
              "#pragma once", *sorted(include_set), "", "namespace mr::generated {"]
    return "\n".join(header + body + ["}  // namespace mr::generated", ""])


def render_chassis(config: dict, module_name: str) -> str:
    plan = plan_chassis(config, module_name)
    chassis = plan["module"]
    selected = plan["wheels"]
    period = plan["period_us"]
    model = {"mecanum": "kMecanum", "front_omni_rear_mecanum": "kFrontOmniRearMecanum"}[chassis["model"]]
    lines = [
        "class ChassisAssembly final {",
        " public:",
        f"  static constexpr uint32_t kPeriodUs = {period}u;",
        f"  static constexpr float kDtSeconds = {_f(period / 1000000)};",
        f"  static constexpr bool kAllowMotorTx = {str(config['runtime']['allow_motor_tx']).lower()};",
        # These values are schema-constrained identifiers, not arbitrary C++ text.
        f'  static constexpr const char* kLogicalCan = "{plan["bus"]}";',
        f'  static constexpr const char* kBoardCan = "{plan["binding"]}";',
        "  explicit ChassisAssembly(mr::motor::CanPort& port)",
        "      : commands(port),",
    ]
    for index, (_, _, motor) in enumerate(selected):
        lines.append(f"        motor_{index}(MotorConfig{index}(port, commands), {{{_f(motor['external_ratio'])}}}),")
    for index, (_, wheel, _) in enumerate(selected):
        ending = " {}" if index == len(selected) - 1 else ","
        lines.append(f"        wheel_{index}(motor_{index}, {_f(wheel['radius_m'])}){ending}")
    lines += ["  ChassisAssembly(const ChassisAssembly&) = delete;",
              "  ChassisAssembly& operator=(const ChassisAssembly&) = delete;",
              "  mr::motor::RmCommandBus commands;"]
    aliases = {"M2006": "RmM2006Motor", "M3508": "RmM3508Motor", "M6020": "RmM6020Motor"}
    for index, (_, _, motor) in enumerate(selected):
        lines.append(f"  mr::motor::{aliases[motor['model']]} motor_{index};")
    for index in range(4):
        lines.append(f"  mr::wheel::MotorWheel<decltype(motor_{index})> wheel_{index};")
    lines += ["  mr::module::chassis::Chassis chassis;",
              "  static mr::module::chassis::ChassisConfig Config() {",
              "    mr::module::chassis::ChassisConfig config;",
              f"    config.kinematics.model = mr::robotics::chassis::ChassisModel::{model};",
              f"    config.kinematics.geometry = {{{_f(chassis['geometry']['half_wheelbase_m'])}, {_f(chassis['geometry']['half_trackwidth_m'])}}};",
              f"    config.max_body_velocity = {{{_f(chassis['limits']['vx_mps'])}, {_f(chassis['limits']['vy_mps'])}, {_f(chassis['limits']['wz_rad_s'])}}};",
              f"    config.max_wheel_speed_mps = {_f(chassis['limits']['max_wheel_speed_mps'])};",
              f"    config.feedback_timeout_ms = {chassis['feedback_timeout_ms']}u;",
              f"    config.wheel_velocity_pid.gains = {{{_f(chassis['wheel_velocity_pid']['k'])}, {_f(chassis['wheel_velocity_pid']['p'])}, {_f(chassis['wheel_velocity_pid']['i'])}, {_f(chassis['wheel_velocity_pid']['d'])}}};",
              f"    config.wheel_velocity_pid.output_limit = mr::comp::cntlr::SymmetricLimit::Abs({_f(chassis['wheel_velocity_pid']['output_limit_nm'])});",
              f"    config.wheel_velocity_pid.control_freq_hz = {_f(1000000 / period)};",
              "    return config;",
              "  }",
              "  mr::module::chassis::ChassisStatus Register() {",
              "    return chassis.Register(Config(), {&wheel_0, &wheel_1, &wheel_2, &wheel_3});",
              "  }", " private:"]
    for index, (_, _, motor) in enumerate(selected):
        lines += [
            f"  static mr::motor::MotorInstanceConfig<mr::motor::MotorKind::RM> MotorConfig{index}(mr::motor::CanPort& port, mr::motor::RmCommandBus& commands) {{",
            "    mr::motor::MotorInstanceConfig<mr::motor::MotorKind::RM> config;",
            "    config.port = &port;", "    config.commands = &commands;",
            f"    config.id = 0x{motor['feedback_id']:03x};",
            f"    config.reverse = {str(motor['reverse']).lower()};",
            f"    config.feedback_timeout_ms = {motor['feedback_timeout_ms']}u;",
            "    return config;", "  }"]
    lines += ["};", ""]
    return "\n".join(lines)


def render_cmake(config: dict, module_name: str, task_name: str | None = None) -> str:
    plans = plan_modules(config, module_name)
    plan = plans[0][1]
    targets = list(dict.fromkeys(target for _, item in plans for target in item["targets"]))
    return ("# Generated by mrctl. Include in a function scope via MrRobot.cmake.\n"
            'set(MR_GENERATED_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}")\n'
            'set(MR_GENERATED_TARGETS ' + " ".join(targets) + ')\n'
            'set(MR_GENERATED_TASK_SOURCES ' +
            ('"${CMAKE_CURRENT_LIST_DIR}/robot_tasks.cpp"' if task_name else '') + ')\n'
            f'set(MR_GENERATED_CAN_BINDING {plan["binding"]})\n'
            f'set(MR_GENERATED_BOARD_PROFILE {config["board"]["profile"]})\n')


def _atomic_write(path: Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(prefix=".mrctl-", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp_name, path)
    except BaseException:
        try:
            os.unlink(temp_name)
        except FileNotFoundError:
            pass
        raise


def generate(config_path: Path, output_dir: Path, module_name: str,
             task_name: str | None = None) -> Path:
    config_path = config_path.resolve()
    for part in (output_dir.absolute(), *output_dir.absolute().parents):
        if part.is_symlink() or (hasattr(part, "is_junction") and part.is_junction()):
            raise ValueError("generated output must not traverse symlinks or junctions")
    output_dir = output_dir.resolve()
    # The output folder has an exact role, not an arbitrary user directory.
    if output_dir.name != "generated":
        raise ValueError("output directory must be named generated")
    if output_dir.is_symlink() or output_dir == config_path.parent:
        raise ValueError("generated directory must be separate from the input config")
    if output_dir in config_path.parents:
        raise ValueError("input config must not be inside generated")
    config = load_config(config_path)
    library = (config_path.parent / config["project"]["library"]).resolve()
    if library != ROOT:
        raise ValueError(f"project.library must resolve to the generator's library checkout: {ROOT}")
    contents = {GENERATED_HEADER: render_config(config, module_name).encode("utf-8"),
                "robot.cmake": render_cmake(config, module_name, task_name).encode("utf-8")}
    if task_name:
        from tools.task_generator import render_task_files
        names = [name for name, _ in plan_modules(config, module_name)]
        contents.update({name: value.encode("utf-8") for name, value in
                         render_task_files(config, task_name, names).items()})
    manifest_path = output_dir / MANIFEST
    manifest = {"generator": "mrctl-robot-v2", "files": {}}
    if manifest_path.is_symlink():
        raise ValueError("generated manifest must not be a symlink")
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if not isinstance(manifest, dict) or manifest.get("generator") != "mrctl-robot-v2" or not isinstance(manifest.get("files"), dict):
            raise ValueError(f"unrecognized generated manifest: {manifest_path}")
    # Complete conflict checks before writing any files. Never overwrite hand edits.
    for name in contents:
        path = output_dir / name
        previous = manifest["files"].get(name)
        if path.is_symlink() or (path.exists() and (
                not previous or hashlib.sha256(path.read_bytes()).hexdigest() != previous)):
            raise ValueError(f"refusing to overwrite unowned or modified file: {path}")
    # Keep ownership of optional task files when generation is disabled; they
    # are no longer compiled, but can safely be regenerated when enabled again.
    hashes = dict(manifest["files"])
    for name, content in contents.items():
        hashes[name] = hashlib.sha256(content).hexdigest()
        path = output_dir / name
        if not path.exists() or path.read_bytes() != content:
            _atomic_write(path, content)
    manifest = {"generator": "mrctl-robot-v2", "module": module_name, "files": hashes}
    manifest_content = (json.dumps(manifest, indent=2) + "\n").encode("utf-8")
    if not manifest_path.exists() or manifest_path.read_bytes() != manifest_content:
        _atomic_write(manifest_path, manifest_content)
    return output_dir / GENERATED_HEADER
