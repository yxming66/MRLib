"""Compile emitted task wiring and exercise its CMSIS call contract."""
import copy
import os
from pathlib import Path
import shutil
import subprocess
import sys
import unittest
import uuid

import yaml
from tools.robot_config import generate, load_config

ROOT = Path(__file__).resolve().parents[1]


class TaskGenerationTests(unittest.TestCase):
    def setUp(self):
        self.project = ROOT / "build" / ("task-gen-" + uuid.uuid4().hex[:8])
        self.project.mkdir(parents=True)
        self.addCleanup(self.cleanup)
        self.config = load_config(ROOT / "examples/chassis_task/config.yaml")
        self.config["project"]["library"] = os.path.relpath(ROOT, self.project).replace("\\", "/")
        self.path = self.project / "config.yaml"
        self.output = self.project / "generated"
        self.write()

    def cleanup(self):
        resolved = self.project.resolve()
        if resolved.parent != (ROOT / "build").resolve() or self.project.is_symlink():
            raise ValueError("test cleanup escaped workspace")
        shutil.rmtree(resolved)

    def write(self):
        self.path.write_text(yaml.safe_dump(self.config, sort_keys=False), encoding="utf-8")

    def test_task_wiring_runtime_and_yaml_rebuild(self):
        self.project.joinpath("CMakeLists.txt").write_text(f'''cmake_minimum_required(VERSION 3.22)
project(task_probe LANGUAGES C CXX)
set(MR_TESTLIB_BUILD_HOST_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory("{ROOT.as_posix()}" mr_library)
include("{ROOT.as_posix()}/cmake/MrRobot.cmake")
add_library(board_probe INTERFACE)
mr_register_board_adapter(TARGET board_probe PROFILE h723_zzrobot CAN_ENDPOINTS BSP_CAN_1)
add_executable(probe "{ROOT.as_posix()}/tests/task/periodic_task_probe.cpp")
target_include_directories(probe PRIVATE "{ROOT.as_posix()}/tests/stubs/task")
target_compile_options(probe PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
mr_target_robot_config(TARGET probe CONFIG "${{CMAKE_CURRENT_SOURCE_DIR}}/config.yaml"
  MODULE chassis TASK chassis BOARD_TARGET board_probe PYTHON "{Path(sys.executable).as_posix()}")
''', encoding="utf-8")
        build = self.project / "compiled"
        exe = build / ("probe.exe" if os.name == "nt" else "probe")
        command = ["cmake", "-S", str(self.project), "-B", str(build)]
        if os.name == "nt":
            command += ["-G", "MinGW Makefiles",
                        "-DCMAKE_MAKE_PROGRAM=D:/Drivers/MSYS2/ucrt64/bin/mingw32-make.exe",
                        "-DCMAKE_C_COMPILER=D:/Drivers/MSYS2/ucrt64/bin/gcc.exe",
                        "-DCMAKE_CXX_COMPILER=D:/Drivers/MSYS2/ucrt64/bin/g++.exe"]
        self.run_command(command)
        for period, stack in ((2000, 4096), (4000, 8192)):
            self.config["tasks"]["chassis"].update(period_us=period, stack_bytes=stack)
            self.write()
            # Normal build must regenerate task parameters after YAML changes.
            self.run_command(["cmake", "--build", str(build), "--parallel", "4"])
            self.run_command([str(exe), str(period), str(stack)])
        generate(self.path, self.output, "chassis", "chassis")
        before = (self.output / "robot_tasks.hpp").read_bytes()
        generate(self.path, self.output, "chassis")
        self.assertNotIn("robot_tasks.cpp", (self.output / "robot.cmake").read_text())
        generate(self.path, self.output, "chassis", "chassis")
        self.assertEqual(before, (self.output / "robot_tasks.hpp").read_bytes())
        (self.output / "robot_tasks.hpp").write_text("// manual", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "refusing to overwrite"):
            generate(self.path, self.output, "chassis", "chassis")

    def run_command(self, command):
        result = subprocess.run(command, text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_invalid_task_never_writes(self):
        with self.assertRaisesRegex(ValueError, "not present"):
            generate(self.path, self.output, "chassis", "missing")
        self.config["tasks"]["chassis"]["entry"] = "app.bad();"
        self.write()
        with self.assertRaisesRegex(ValueError, "dotted identifier"):
            generate(self.path, self.output, "chassis", "chassis")
        self.assertFalse(self.output.exists())
        config = load_config(ROOT / "examples/h7_integration/config.yaml")
        config["project"]["library"] = self.config["project"]["library"]
        self.config = copy.deepcopy(config)
        self.write()
        with self.assertRaisesRegex(ValueError, "exactly the selected modules"):
            generate(self.path, self.output, "chassis", "robot")
        with self.assertRaisesRegex(ValueError, "communication services"):
            generate(self.path, self.output, "all", "robot")
        self.assertFalse(self.output.exists())


if __name__ == "__main__":
    unittest.main()
