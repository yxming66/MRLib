#!/usr/bin/env python3
"""Standard-library v1 host CLI and optional-dependency v2 assembly generator."""

from __future__ import annotations

import argparse
import json
from pathlib import Path, PureWindowsPath
import re
import shutil
import subprocess
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
TARGET_CATALOG_PATH = ROOT / "cmake" / "targets.json"


def load_target_catalog() -> dict[str, dict[str, Any]]:
    """Load and validate the public build catalog shared with CMake."""
    try:
        document = json.loads(TARGET_CATALOG_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(f"unable to read target catalog {TARGET_CATALOG_PATH}: {error}") from error
    if not isinstance(document, dict) or document.get("version") != 1:
        raise RuntimeError("target catalog must be a version 1 JSON object")
    entries = document.get("targets")
    if not isinstance(entries, list) or not entries:
        raise RuntimeError("target catalog must contain a non-empty 'targets' list")

    catalog: dict[str, dict[str, Any]] = {}
    aliases: set[str] = set()
    for entry in entries:
        if not isinstance(entry, dict):
            raise RuntimeError("each target catalog entry must be an object")
        required_fields = {
            "name",
            "alias",
            "type",
            "include_dirs",
            "compile_features",
            "header",
            "dependencies",
        }
        optional_fields = {"sources", "build_by_default"}
        missing_fields = required_fields - set(entry)
        unknown_fields = set(entry) - required_fields - optional_fields
        if missing_fields or unknown_fields:
            raise RuntimeError(
                f"invalid fields for target catalog entry: missing={sorted(missing_fields)}, "
                f"unknown={sorted(unknown_fields)}"
            )
        name, alias, header = entry.get("name"), entry.get("alias"), entry.get("header")
        if not isinstance(name, str) or not re.fullmatch(r"mr_[a-z0-9_]+", name):
            raise RuntimeError(f"invalid target name in catalog: {name!r}")
        if name in catalog:
            raise RuntimeError(f"duplicate target name in catalog: {name}")
        if not isinstance(alias, str) or not re.fullmatch(r"mr::[a-z0-9_]+", alias):
            raise RuntimeError(f"invalid public alias for {name}: {alias!r}")
        if alias in aliases:
            raise RuntimeError(f"duplicate target alias in catalog: {alias}")
        if (
            not isinstance(header, str)
            or not re.fullmatch(r"[A-Za-z0-9_./-]+", header)
            or Path(header).is_absolute()
            or PureWindowsPath(header).is_absolute()
            or ".." in Path(header).parts
            or ".." in PureWindowsPath(header).parts
        ):
            raise RuntimeError(f"invalid public header for {name}: {header!r}")
        if not (ROOT / "User" / header).is_file():
            raise RuntimeError(f"public header for {name} does not exist: User/{header}")
        if entry.get("type") not in ("INTERFACE", "STATIC"):
            raise RuntimeError(f"unsupported target type for {name}: {entry.get('type')!r}")
        if "build_by_default" in entry and type(entry["build_by_default"]) is not bool:
            raise RuntimeError(f"build_by_default for {name} must be true or false")
        if entry.get("build_by_default") and entry["type"] != "STATIC":
            raise RuntimeError(f"build_by_default only applies to static target {name}")
        for key in ("include_dirs", "compile_features", "dependencies"):
            values = entry.get(key)
            if not isinstance(values, list) or any(not isinstance(value, str) for value in values):
                raise RuntimeError(f"target {name} must define a string list '{key}'")
            if len(values) != len(set(values)):
                raise RuntimeError(f"target {name} repeats an item in '{key}'")
        if any(
            not re.fullmatch(r"(?:c|cxx)_std_[0-9]+", feature)
            for feature in entry["compile_features"]
        ):
            raise RuntimeError(f"unsupported compile feature for {name}")
        for include_dir in entry["include_dirs"]:
            if (
                not re.fullmatch(r"[A-Za-z0-9_./-]+", include_dir)
                or Path(include_dir).is_absolute()
                or PureWindowsPath(include_dir).is_absolute()
                or ".." in Path(include_dir).parts
                or ".." in PureWindowsPath(include_dir).parts
            ):
                raise RuntimeError(f"invalid include directory for {name}: {include_dir!r}")
            if not (ROOT / include_dir).is_dir():
                raise RuntimeError(f"include directory for {name} does not exist: {include_dir}")
        if entry["type"] == "STATIC":
            sources = entry.get("sources")
            if not isinstance(sources, list) or not sources:
                raise RuntimeError(f"static target {name} must define source files")
            for source in sources:
                if (
                    not isinstance(source, str)
                    or not re.fullmatch(r"[A-Za-z0-9_./-]+", source)
                    or Path(source).is_absolute()
                    or PureWindowsPath(source).is_absolute()
                    or ".." in Path(source).parts
                    or ".." in PureWindowsPath(source).parts
                ):
                    raise RuntimeError(f"invalid source path for {name}: {source!r}")
                if not (ROOT / source).is_file():
                    raise RuntimeError(f"source for {name} does not exist: {source}")
        elif "sources" in entry:
            raise RuntimeError(f"interface target {name} must not define source files")
        catalog[name] = entry
        aliases.add(alias)

    for name, entry in catalog.items():
        for dependency in entry["dependencies"]:
            if dependency not in catalog:
                raise RuntimeError(f"target {name} depends on unknown target {dependency}")
            if dependency == name:
                raise RuntimeError(f"target {name} cannot depend on itself")
    return catalog


try:
    TARGET_CATALOG = load_target_catalog()
except RuntimeError as error:
    raise SystemExit(str(error)) from error

TARGET_HEADERS = {name: entry["header"] for name, entry in TARGET_CATALOG.items()}


def resolve_targets(targets: list[str]) -> list[str]:
    """Return selected targets and their transitive dependencies once each."""
    resolved: list[str] = []
    visiting: set[str] = set()
    complete: set[str] = set()

    def visit(name: str) -> None:
        if name in complete:
            return
        if name in visiting:
            raise ValueError(f"target catalog contains a dependency cycle at {name}")
        visiting.add(name)
        for dependency in TARGET_CATALOG[name]["dependencies"]:
            visit(dependency)
        visiting.remove(name)
        complete.add(name)
        resolved.append(name)

    for target in targets:
        visit(target)
    return resolved


for _target_name in TARGET_CATALOG:
    try:
        resolve_targets([_target_name])
    except ValueError as error:
        raise SystemExit(str(error)) from error


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
project({name} LANGUAGES C CXX)

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
    resolved_targets = resolve_targets(project["targets"])
    selected = set(project["targets"])
    dependencies = [target for target in resolved_targets if target not in selected]
    if dependencies:
        print("resolved dependencies: " + ", ".join(dependencies), flush=True)
    probe = "".join(
        f'#include "{TARGET_HEADERS[target]}"\n' for target in resolved_targets
    )
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
    targets = ";".join(TARGET_CATALOG[target]["alias"] for target in project["targets"])
    headers = ";".join(TARGET_HEADERS[target] for target in resolved_targets)
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
    generate_command = commands.add_parser("generate", help="generate selected v2 chassis assembly")
    generate_command.add_argument("config", type=Path)
    generate_command.add_argument("--module", required=True, help="explicitly selected chassis module")
    generate_command.add_argument("--output-dir", type=Path, default=None)
    for name in ("configure", "build"):
        command = commands.add_parser(name)
        command.add_argument("directory", type=Path)
        command.add_argument("--build-dir", type=Path, default=None)
    args = parser.parse_args()
    if args.command == "generate":
        # Keep optional YAML/Schema packages out of the existing v1 CLI path.
        sys.path.insert(0, str(ROOT))
        try:
            from tools.robot_config import generate
        except ImportError as error:
            parser.error(f"v2 generation requires tools/requirements-config.txt: {error}")
        try:
            output = generate(args.config, args.output_dir or args.config.parent / "generated",
                              args.module)
        except (OSError, ValueError) as error:
            parser.error(str(error))
        print(f"generated chassis module {args.module}: {output}")
        print("scope: selected RM motors/wheels/chassis only; other modules and tasks remain app-owned")
        return 0
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
