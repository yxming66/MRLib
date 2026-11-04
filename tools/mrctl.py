#!/usr/bin/env python3
"""Dependency-free project generator and host configure/build front end."""

from __future__ import annotations

import argparse
import json
from pathlib import Path, PureWindowsPath
import re
import shutil
import subprocess
import sys
from typing import Any


TARGET_HEADERS = {
    "mr_component_math": "component/math/scalar.hpp",
    "mr_component_filter": "component/filter/filter.hpp",
    "mr_component_controller": "component/controller/controller.hpp",
    "mr_component_trajectory": "component/trajectory/trajectory_types.hpp",
    "mr_robotics_core": "robotics/core/rotation.hpp",
    "mr_robotics_chassis": "robotics/chassis/chassis_kinematics.hpp",
    "mr_device_wheel_interface": "device/wheel/wheel_interface.hpp",
    "mr_module_chassis": "module/chassis/chassis.hpp",
    "mr_robotics_arm": "robotics/arm/arm.hpp",
}


def parse_scalar(value: str) -> Any:
    value = value.strip()
    if not value:
        return ""
    if (value.startswith('"') and value.endswith('"')) or (
        value.startswith("'") and value.endswith("'")
    ):
        return value[1:-1]
    if value.lower() in ("true", "false"):
        return value.lower() == "true"
    try:
        return int(value)
    except ValueError:
        return value


def parse_config(path: Path) -> dict[str, Any]:
    """Read the documented restricted YAML mapping and block-list syntax."""
    config: dict[str, Any] = {}
    project: dict[str, Any] | None = None
    list_key: str | None = None
    for number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not raw.strip() or raw.lstrip().startswith("#"):
            continue
        indent = len(raw) - len(raw.lstrip(" "))
        text = raw.strip()
        if "\t" in raw[: len(raw) - len(raw.lstrip())]:
            raise ValueError(f"tabs are not supported for indentation at {path}:{number}")
        if indent == 0 and text.endswith(":"):
            key = text[:-1]
            if key in config:
                raise ValueError(f"duplicate key '{key}' at {path}:{number}")
            project = {} if key == "project" else None
            config[key] = project if project is not None else {}
            list_key = None
        elif indent == 0 and ":" in text:
            key, value = text.split(":", 1)
            if key in config:
                raise ValueError(f"duplicate key '{key}' at {path}:{number}")
            config[key] = parse_scalar(value)
            project = None
            list_key = None
        elif indent == 2 and project is not None and ":" in text:
            key, value = text.split(":", 1)
            key = key.strip()
            if key in project:
                raise ValueError(f"duplicate key '{key}' at {path}:{number}")
            if value.strip():
                project[key] = parse_scalar(value)
                list_key = None
            else:
                project[key] = []
                list_key = key
        elif indent == 4 and project is not None and text.startswith("-") and list_key:
            project[list_key].append(parse_scalar(text[1:].strip()))
        else:
            raise ValueError(f"unsupported YAML at {path}:{number}: {raw}")
    return config


def read_config(project_dir: Path) -> dict[str, Any]:
    config_path = project_dir / "project.yaml"
    if not config_path.is_file():
        raise SystemExit(f"missing {config_path}; run 'mrctl.py init <dir>' first")
    try:
        config = parse_config(config_path)
    except ValueError as error:
        raise SystemExit(str(error)) from error
    if (
        set(config) != {"version", "project"}
        or type(config["version"]) is not int
        or config["version"] != 1
    ):
        raise SystemExit("project.yaml must contain 'version: 1' and a project mapping")
    project = config["project"]
    if not isinstance(project, dict) or set(project) != {"name", "library", "targets"}:
        raise SystemExit("project must contain exactly: name, library, targets")
    name, library, targets = project["name"], project["library"], project["targets"]
    if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.+-]*", name):
        raise SystemExit("project.name must be a simple CMake target name")
    if (
        not isinstance(library, str)
        or not library
        or Path(library).is_absolute()
        or PureWindowsPath(library).is_absolute()
        or any(ord(char) < 32 or char in ';"$' for char in library)
    ):
        raise SystemExit("project.library must be a relative path without CMake expressions")
    if not isinstance(targets, list) or not targets or any(
        not isinstance(target, str) or target not in TARGET_HEADERS for target in targets
    ):
        allowed = ", ".join(sorted(TARGET_HEADERS))
        raise SystemExit(f"project.targets must list known targets: {allowed}")
    if len(targets) != len(set(targets)):
        raise SystemExit("project.targets must not repeat targets")
    return project


def write_new(path: Path, content: str) -> bool:
    """Create a starter file without replacing existing user content."""
    if path.exists():
        return False
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with path.open("x", encoding="utf-8", newline="\n") as output:
            output.write(content)
        return True
    except FileExistsError:
        return False


def render_cmake(project: dict[str, Any]) -> str:
    name, library = project["name"], project["library"].replace("\\", "/")
    return f'''cmake_minimum_required(VERSION 3.22)
project({name} LANGUAGES CXX)

# Relative to this project directory. Set --library to your vendored submodule.
set(MR_TESTLIB_DIR "{library}")
add_subdirectory("${{CMAKE_CURRENT_SOURCE_DIR}}/${{MR_TESTLIB_DIR}}"
                 "${{CMAKE_BINARY_DIR}}/mr_testlib")

if(NOT MR_TESTLIB_PROJECT_TARGETS OR NOT MR_TESTLIB_PROJECT_HEADERS)
    message(FATAL_ERROR "Run mrctl.py configure <project-dir> first.")
endif()

set(_mr_probe_file "${{CMAKE_BINARY_DIR}}/mrctl_compile_check.cpp")
if(NOT EXISTS "${{_mr_probe_file}}")
    message(FATAL_ERROR "Run mrctl.py configure <project-dir> to generate the selected API compile check.")
endif()

add_executable({name} src/main.cpp "${{_mr_probe_file}}")
target_compile_features({name} PRIVATE cxx_std_17)
target_link_libraries({name} PRIVATE ${{MR_TESTLIB_PROJECT_TARGETS}})
'''


def render_main() -> str:
    return "// Replace this stub with the board application entry point.\n\nint main() {\n  return 0;\n}\n"


def init_project(project_dir: Path, name: str, library: str, targets: list[str]) -> int:
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.+-]*", name):
        raise SystemExit("--name must be a simple CMake target name")
    if (
        not library
        or Path(library).is_absolute()
        or PureWindowsPath(library).is_absolute()
        or any(ord(char) < 32 or char in ';"$' for char in library)
    ):
        raise SystemExit("--library must be a relative path without CMake expressions")
    unknown = [target for target in targets if target not in TARGET_HEADERS]
    if not targets or unknown or len(targets) != len(set(targets)):
        raise SystemExit("unknown, empty or repeated targets: " + ", ".join(unknown))
    project_dir = project_dir.resolve()
    project = {"name": name, "library": library.replace("\\", "/"), "targets": targets}
    if (project_dir / "project.yaml").exists():
        existing = read_config(project_dir)
        if existing != project:
            raise SystemExit(
                f"{project_dir / 'project.yaml'} already defines this project; edit it and run configure"
            )
        project = existing
    project_dir.mkdir(parents=True, exist_ok=True)
    schema_path = f"{project['library'].rstrip('/')}/docs/project.schema.json"
    config = f"# yaml-language-server: $schema={schema_path}\nversion: 1\nproject:\n"
    config += f"  name: {json.dumps(name)}\n  library: {json.dumps(project['library'])}\n  targets:\n"
    config += "".join(f"    - {target}\n" for target in targets)
    created = [
        (project_dir / "project.yaml", config),
        (project_dir / "CMakeLists.txt", render_cmake(project)),
        (project_dir / "src" / "main.cpp", render_main()),
        (project_dir / ".gitignore", "build/\n"),
    ]
    written = [str(path.relative_to(project_dir)) for path, content in created if write_new(path, content)]
    if not written:
        print(f"no files written; {project_dir} already contains the project files")
        return 0
    print("created:\n" + "\n".join(f"  {item}" for item in written))
    return 0


def run(command: list[str]) -> int:
    print("$ " + subprocess.list2cmdline(command), flush=True)
    try:
        return subprocess.run(command, check=False).returncode
    except OSError as error:
        raise SystemExit(f"unable to run {command[0]}: {error}") from error


def configure(project_dir: Path, build_dir: Path) -> int:
    project = read_config(project_dir)
    if not (project_dir / "CMakeLists.txt").is_file() or not (project_dir / "src" / "main.cpp").is_file():
        raise SystemExit("project files are incomplete; restore the missing files and run init again")
    library_dir = (project_dir / project["library"]).resolve()
    if not (library_dir / "CMakeLists.txt").is_file():
        raise SystemExit(f"project.library does not point to an MR_TestLib checkout: {library_dir}")
    build_dir.mkdir(parents=True, exist_ok=True)
    probe = "".join(f'#include "{TARGET_HEADERS[target]}"\n' for target in project["targets"])
    probe += "void mr_project_selected_headers_compile_check() {}\n"
    (build_dir / "mrctl_compile_check.cpp").write_text(probe, encoding="utf-8", newline="\n")
    command = ["cmake", "-S", str(project_dir), "-B", str(build_dir)]
    cache_file = build_dir / "CMakeCache.txt"
    generator = None
    if cache_file.is_file():
        for line in cache_file.read_text(encoding="utf-8", errors="replace").splitlines():
            if line.startswith("CMAKE_GENERATOR:INTERNAL="):
                generator = line.partition("=")[2]
                break
    elif shutil.which("ninja"):
        generator = "Ninja"
    if generator:
        command += ["-G", generator]
    targets = ";".join(f"mr::{target.removeprefix('mr_')}" for target in project["targets"])
    headers = ";".join(TARGET_HEADERS[target] for target in project["targets"])
    command += [
        f"-DMR_TESTLIB_PROJECT_TARGETS:STRING={targets}",
        f"-DMR_TESTLIB_PROJECT_HEADERS:STRING={headers}",
    ]
    return run(command)


def build(project_dir: Path, build_dir: Path) -> int:
    project = read_config(project_dir)
    print(f"building {project['name']} ({', '.join(project['targets'])})", flush=True)
    configured = configure(project_dir, build_dir)
    if configured != 0:
        return configured
    return run(["cmake", "--build", str(build_dir)])


def main() -> int:
    parser = argparse.ArgumentParser(prog="mrctl", description="Create and build an MR_TestLib project")
    commands = parser.add_subparsers(dest="command", required=True)
    init = commands.add_parser("init", help="create a project without overwriting existing files")
    init.add_argument("directory", type=Path)
    init.add_argument("--name", required=True)
    init.add_argument("--library", required=True, help="MR_TestLib path relative to the new project")
    init.add_argument("--targets", required=True, help="comma-separated CMake target names")
    for name in ("configure", "build"):
        command = commands.add_parser(name)
        command.add_argument("directory", type=Path)
        command.add_argument("--build-dir", type=Path, default=None)
    args = parser.parse_args()
    if args.command == "init":
        targets = [item.strip() for item in args.targets.split(",") if item.strip()]
        return init_project(args.directory, args.name, args.library, targets)
    project_dir = args.directory.resolve()
    build_dir = (args.build_dir or project_dir / "build").resolve()
    if args.command == "configure":
        return configure(project_dir, build_dir)
    return build(project_dir, build_dir)


if __name__ == "__main__":
    sys.exit(main())
