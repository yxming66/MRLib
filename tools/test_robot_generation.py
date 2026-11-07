"""Exercise generated assembly, dependency closure and regeneration boundaries."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import uuid
import unittest

import yaml

from tools.robot_config import generate, load_config

ROOT = Path(__file__).resolve().parents[1]


class RobotGenerationTests(unittest.TestCase):
    def setUp(self):
        (ROOT / "build").mkdir(exist_ok=True)
        # Use inherited Windows ACLs; tempfile's mode=0700 directory can become
        # inaccessible to the managed desktop sandbox identity.
        self.project = ROOT / "build" / ("robot-gen-" + uuid.uuid4().hex[:8])
        self.project.mkdir()
        def cleanup():
            resolved = self.project.resolve()
            if resolved.parent != (ROOT / "build").resolve() or self.project.is_symlink():
                raise ValueError("test cleanup escaped its workspace")
            shutil.rmtree(resolved)
        self.addCleanup(cleanup)
        self.path = self.project / "config.yaml"
        self.output = self.project / "generated"
        self.config = load_config(ROOT / "examples/h7_integration/config.yaml")
        self.config["project"]["library"] = os.path.relpath(ROOT, self.project).replace("\\", "/")
        self.write_config()

    def write_config(self):
        self.path.write_text(yaml.safe_dump(self.config, sort_keys=False), encoding="utf-8")

    def test_idempotent_and_preserves_handwritten_files(self):
        app = self.project / "app.cpp"
        app.write_text("// user code\n", encoding="utf-8")
        header = generate(self.path, self.output, "chassis")
        before = header.read_bytes(), header.stat().st_mtime_ns
        generate(self.path, self.output, "chassis")
        self.assertEqual(before, (header.read_bytes(), header.stat().st_mtime_ns))
        self.assertEqual(app.read_text(), "// user code\n")

    def test_regeneration_changes_parameters_and_order(self):
        header = generate(self.path, self.output, "chassis")
        previous = header.read_bytes()
        self.config["modules"]["chassis"]["wheels"].reverse()
        self.config["devices"]["wheels"]["rear_right"]["radius_m"] = .125
        self.config["tasks"]["robot"]["period_us"] = 2000
        self.write_config()
        generate(self.path, self.output, "chassis")
        self.assertNotEqual(previous, header.read_bytes())
        self.assertIn("wheel_0(motor_0, 0.125f)", header.read_text())
        self.assertIn("control_freq_hz = 500.0f", header.read_text())

    def test_modified_output_and_unowned_output_are_preserved(self):
        header = generate(self.path, self.output, "chassis")
        cmake = self.output / "robot.cmake"
        previous = cmake.read_bytes()
        header.write_text("// manual edit\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "refusing to overwrite"):
            generate(self.path, self.output, "chassis")
        self.assertEqual(header.read_text(), "// manual edit\n")
        self.assertEqual(cmake.read_bytes(), previous)
        (self.output / ".mrctl-manifest.json").unlink()
        with self.assertRaisesRegex(ValueError, "refusing to overwrite"):
            generate(self.path, self.output, "chassis")

    def test_invalid_config_and_scope_never_write(self):
        with self.assertRaisesRegex(ValueError, "not present"):
            generate(self.path, self.output, "missing")
        self.assertFalse(self.output.exists())
        self.config["devices"]["motors"]["arm_0"]["can_id"] = 1
        self.write_config()
        with self.assertRaisesRegex(ValueError, "conflict"):
            generate(self.path, self.output, "chassis")
        self.assertFalse(self.output.exists())

    def test_arm_only_and_combined_selection(self):
        header = generate(self.path, self.output, "arm")
        self.assertIn("class ArmAssembly", header.read_text())
        self.assertNotIn("class ChassisAssembly", header.read_text())
        self.assertNotIn("mr::module_chassis", (self.output / "robot.cmake").read_text())
        generate(self.path, self.output, "all")
        before = header.read_bytes()
        self.assertIn("class ChassisAssembly", header.read_text())
        generate(self.path, self.output, "chassis,arm")
        self.assertEqual(before, header.read_bytes())
        header.write_text("// manual edit", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "refusing to overwrite"):
            generate(self.path, self.output, "all")

    def test_arm_bus_and_numeric_boundaries(self):
        self.config["board"]["bindings"]["can"]["second_bus"] = "BSP_CAN_2"
        self.config["devices"]["motors"]["arm_0"]["bus"] = "second_bus"
        self.write_config()
        with self.assertRaisesRegex(ValueError, "one CAN port"):
            generate(self.path, self.output, "arm")
        for name in ("arm_1", "arm_2"):
            self.config["devices"]["motors"][name]["bus"] = "second_bus"
        self.write_config()
        with self.assertRaisesRegex(ValueError, "shared CAN port"):
            generate(self.path, self.output, "all")
        self.config["tasks"]["robot"]["period_us"] = 100001
        self.write_config()
        with self.assertRaisesRegex(ValueError, "period must be"):
            generate(self.path, self.output, "arm")
        self.config["tasks"]["robot"]["period_us"] = 1000
        self.config["devices"]["joints"]["shoulder"]["zero_offset_rad"] = 1e100
        self.write_config()
        with self.assertRaisesRegex(ValueError, "float32"):
            generate(self.path, self.output, "arm")
        self.assertFalse(self.output.exists())

    def test_output_role_and_library_mismatch(self):
        with self.assertRaisesRegex(ValueError, "named generated"):
            generate(self.path, self.project / "app", "chassis")
        self.config["project"]["library"] = "missing-library"
        self.write_config()
        with self.assertRaisesRegex(ValueError, "library checkout"):
            generate(self.path, self.output, "chassis")

    def test_mixed_bus_is_explicitly_unsupported(self):
        self.config["board"]["bindings"]["can"]["second_bus"] = "BSP_CAN_2"
        self.config["devices"]["motors"]["wheel_fr"]["bus"] = "second_bus"
        self.write_config()
        with self.assertRaisesRegex(ValueError, "one CAN port"):
            generate(self.path, self.output, "chassis")

    def test_embedded_numeric_limits_fail_before_writing(self):
        self.config["tasks"]["robot"]["period_us"] = 100001
        self.write_config()
        with self.assertRaisesRegex(ValueError, "period must be"):
            generate(self.path, self.output, "chassis")
        self.config["tasks"]["robot"]["period_us"] = 1000
        self.config["devices"]["wheels"]["front_right"]["radius_m"] = 1e100
        self.write_config()
        with self.assertRaisesRegex(ValueError, "float32"):
            generate(self.path, self.output, "chassis")
        self.assertFalse(self.output.exists())

    def run_command(self, args, success=True):
        result = subprocess.run(args, cwd=ROOT, text=True, capture_output=True)
        if success:
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        else:
            self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        return result.stdout + result.stderr

    def test_board_helper_selects_only_declared_capabilities(self):
        cmake = f'''cmake_minimum_required(VERSION 3.22)
project(board_sources NONE)
include("{ROOT.as_posix()}/cmake/MrStm32H7.cmake")
add_library(platform INTERFACE)
mr_add_stm32h7_bsp(TARGET can_board PLATFORM platform PROFILE h723_zzrobot COMPONENTS can)
get_target_property(sources can_board SOURCES)
list(LENGTH sources count)
if(NOT count EQUAL 3)
  message(FATAL_ERROR "can source closure must contain fdcan/time/mm only: ${{sources}}")
endif()
mr_add_stm32h7_bsp(TARGET uart_board PLATFORM platform PROFILE h723_zzrobot COMPONENTS uart)
get_target_property(endpoints uart_board MR_ROBOT_CAN_ENDPOINTS)
if(endpoints)
  message(FATAL_ERROR "uart-only adapter must not claim CAN endpoints")
endif()
# Stop before generating language-dependent OBJECT targets.
message(FATAL_ERROR "capabilities_checked")
'''
        self.project.joinpath("CMakeLists.txt").write_text(cmake, encoding="utf-8")
        command = ["cmake", "-S", str(self.project), "-B", str(self.project / "build")]
        if os.name == "nt":
            command += ["-G", "MinGW Makefiles",
                        "-DCMAKE_MAKE_PROGRAM=D:/Drivers/MSYS2/ucrt64/bin/mingw32-make.exe"]
        log = self.run_command(command, success=False)
        self.assertIn("capabilities_checked", log)
        self.project.joinpath("CMakeLists.txt").write_text(
            f'cmake_minimum_required(VERSION 3.22)\nproject(missing NONE)\n'
            f'include("{ROOT.as_posix()}/cmake/MrRobot.cmake")\n'
            'mr_register_board_adapter()\n', encoding="utf-8")
        log = self.run_command(command, success=False)
        self.assertIn("requires TARGET and PROFILE", log)

    def test_generated_project_compiles_runs_and_rebuilds_from_yaml(self):
        # An independently created project receives only the generated target
        # closure; fake CAN exercises actual MotorT/Chassis registration and output.
        self.config["devices"]["wheels"]["front_right"]["radius_m"] = .125
        self.config["devices"]["motors"]["wheel_fr"]["external_ratio"] = 2.0
        self.config["tasks"]["robot"]["period_us"] = 2000
        self.write_config()
        self.project.joinpath("main.cpp").write_text('''
#include <cassert>
#include <cmath>
#include "robot_config.hpp"
#include "support/fake_can_port.hpp"
int main(int argc, char**) {
  using mr::module::chassis::ChassisStatus;
  FakeCanPort port;
  mr::generated::ChassisAssembly assembly(port);
  assert(assembly.Register() == ChassisStatus::kOk);
  assert(port.sent.empty());
  assert(assembly.wheel_0.RadiusM() == .125f);
  assert(assembly.motor_0.GetInstallConfig().external_ratio == 2.0f);
  assert(assembly.Config().wheel_velocity_pid.control_freq_hz == 500.0f);
  assert(assembly.Config().kinematics.geometry.wheelbase_m == (argc > 1 ? .3f : .4f));
  assert(assembly.chassis.Enable() == ChassisStatus::kOk);
  for (uint32_t id=0x201; id<=0x204; ++id) {
    mr::motor::CanFrame frame; frame.id=id; frame.dlc=8; frame.sequence=1;
    port.Push(frame);
  }
  assembly.commands.BeginCycle();
  assert(assembly.chassis.UpdateFeedback(port.now) == ChassisStatus::kOk);
  assert(assembly.chassis.SetCommand({1,0,0}) == ChassisStatus::kOk);
  assert(assembly.chassis.Step(assembly.kDtSeconds) == ChassisStatus::kOk);
  assert(assembly.chassis.CommitOutput() == ChassisStatus::kOk);
  assert(assembly.commands.Flush() == DEVICE_OK);
  assert(port.sent.size() == 1 && port.sent[0].id == 0x200);
}
''', encoding="utf-8")
        cmake = f'''cmake_minimum_required(VERSION 3.22)
project(robot_generated_probe LANGUAGES C CXX)
set(MR_TESTLIB_BUILD_HOST_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory("{ROOT.as_posix()}" mr_library)
add_library(board_probe INTERFACE)
include("{ROOT.as_posix()}/cmake/MrRobot.cmake")
mr_register_board_adapter(TARGET board_probe PROFILE h723_zzrobot CAN_ENDPOINTS BSP_CAN_1)
add_executable(probe main.cpp)
target_include_directories(probe PRIVATE "{ROOT.as_posix()}/tests")
target_compile_options(probe PRIVATE -Wall -Wextra -Wpedantic -pedantic-errors -UNDEBUG)
mr_target_robot_config(TARGET probe CONFIG "${{CMAKE_CURRENT_SOURCE_DIR}}/config.yaml"
    MODULE chassis BOARD_TARGET board_probe PYTHON "{Path(sys.executable).as_posix()}")
enable_testing()
add_test(NAME generated_probe COMMAND probe)
'''
        self.project.joinpath("CMakeLists.txt").write_text(cmake, encoding="utf-8")
        build = self.project / "build"
        command = ["cmake", "-S", str(self.project), "-B", str(build)]
        if os.name == "nt":
            command += ["-G", "MinGW Makefiles",
                        "-DCMAKE_MAKE_PROGRAM=D:/Drivers/MSYS2/ucrt64/bin/mingw32-make.exe",
                        "-DCMAKE_C_COMPILER=D:/Drivers/MSYS2/ucrt64/bin/gcc.exe",
                        "-DCMAKE_CXX_COMPILER=D:/Drivers/MSYS2/ucrt64/bin/g++.exe"]
        self.run_command(command)
        self.run_command(["cmake", "--build", str(build), "--parallel", "4"])
        self.run_command(["ctest", "--test-dir", str(build), "--output-on-failure"])
        self.assertFalse((build / "mr_library/libmr_robotics_core.a").exists())
        # Change only YAML. CMake's configure-depends must regenerate automatically.
        self.config["modules"]["chassis"]["model"] = "front_omni_rear_mecanum"
        self.config["modules"]["chassis"]["geometry"]["half_wheelbase_m"] = .3
        self.write_config()
        log = self.run_command(["cmake", "--build", str(build), "--parallel", "4"])
        self.assertIn("Robot module 'chassis'", log)
        executable = build / ("probe.exe" if os.name == "nt" else "probe")
        self.run_command([str(executable), "updated"])
        # A board mismatch must fail configure, even with valid YAML.
        self.config["board"]["bindings"]["can"]["motor_can"] = "BSP_CAN_2"
        self.write_config()
        log = self.run_command(["cmake", "--build", str(build)], success=False)
        self.assertIn("does not provide CAN endpoint", log)

    def test_mixed_dm_lz_arm_compiles_runs_and_rebuilds(self):
        arm = self.config["modules"]["arm"]
        arm["joints"] = arm["joints"][:2]
        arm["base_to_world"] = {"translation_m": [.5, 0, 0], "rpy_rad": [0, 0, 1.57079632679]}
        arm["tool_to_last_joint"]["translation_m"] = [0, 0, .25]
        arm["gravity_world_m_s2"] = [0, 0, -9.81]
        arm["joints"][0]["gains"] = {"effort_scale": .75, "kd_nm_s_per_rad": 4,
                                      "kp_nm_per_rad": 40}  # key order must not affect output.
        arm["joints"][1]["axis_in_joint_frame"] = [0, 1, 0]
        arm["joints"][1]["parent_to_joint"] = {
            "translation_m": [.4, 0, 0], "rpy_rad": [1.57079632679, 0, 0]}
        joints = self.config["devices"]["joints"]
        joints["shoulder"]["zero_offset_rad"] = .2
        joints["shoulder"]["limits"]["max_velocity_rad_s"] = 1.5
        joints["shoulder"]["limits"]["max_acceleration_rad_s2"] = 3
        joints["elbow"]["zero_offset_rad"] = -.1
        motors = self.config["devices"]["motors"]
        motors["arm_0"].update(model="J3507", external_ratio=2, reverse=True,
                                feedback_timeout_ms=7, mit={"position_range_rad": 6,
                                "velocity_range_rad_s": 8, "torque_range_nm": 12})
        motors["arm_1"] = {"kind": "lz", "model": "RSO3", "bus": "motor_can", "motor_id": 22,
                           "host_id": 253, "mode": "motion", "reverse": False,
                           "external_ratio": 3, "feedback_timeout_ms": 7}
        self.config["tasks"]["robot"]["period_us"] = 2000
        self.write_config()
        cmake = f'''cmake_minimum_required(VERSION 3.22)
project(arm_generated_probe LANGUAGES C CXX)
set(MR_TESTLIB_BUILD_HOST_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory("{ROOT.as_posix()}" mr_library)
include("{ROOT.as_posix()}/cmake/MrRobot.cmake")
add_library(board_probe INTERFACE)
mr_register_board_adapter(TARGET board_probe PROFILE h723_zzrobot CAN_ENDPOINTS BSP_CAN_1)
add_executable(probe "{ROOT.as_posix()}/tests/generation/arm_probe.cpp")
target_include_directories(probe PRIVATE "{ROOT.as_posix()}/tests")
target_compile_options(probe PRIVATE -Wall -Wextra -Wpedantic -pedantic-errors -Werror -UNDEBUG)
mr_target_robot_config(TARGET probe CONFIG "${{CMAKE_CURRENT_SOURCE_DIR}}/config.yaml"
    MODULE all BOARD_TARGET board_probe PYTHON "{Path(sys.executable).as_posix()}")
'''
        self.project.joinpath("CMakeLists.txt").write_text(cmake, encoding="utf-8")
        build = self.project / "build"
        command = ["cmake", "-S", str(self.project), "-B", str(build)]
        if os.name == "nt":
            command += ["-G", "MinGW Makefiles",
                        "-DCMAKE_MAKE_PROGRAM=D:/Drivers/MSYS2/ucrt64/bin/mingw32-make.exe",
                        "-DCMAKE_C_COMPILER=D:/Drivers/MSYS2/ucrt64/bin/gcc.exe",
                        "-DCMAKE_CXX_COMPILER=D:/Drivers/MSYS2/ucrt64/bin/g++.exe"]
        self.run_command(command)
        self.run_command(["cmake", "--build", str(build), "--parallel", "4"])
        executable = build / ("probe.exe" if os.name == "nt" else "probe")
        self.run_command([str(executable)])
        arm["base_to_world"]["translation_m"][0] = .7
        self.config["tasks"]["robot"]["period_us"] = 4000
        self.write_config()
        log = self.run_command(["cmake", "--build", str(build), "--parallel", "4"])
        self.assertIn("Robot module 'all'", log)
        self.run_command([str(executable), "updated"])
        # An independent one-joint arm must compile without the chassis assembly.
        arm["joints"] = arm["joints"][:1]
        self.write_config()
        self.project.joinpath("main.cpp").write_text('''
#include <cassert>
#include "robot_config.hpp"
#include "support/fake_can_port.hpp"
int main() {
  FakeCanPort port;
  mr::generated::ArmAssembly assembly(port);
  static_assert(decltype(assembly)::kJointCount == 1);
  assert(assembly.Init() == DEVICE_OK);
  assert(port.sent.empty());
}
''', encoding="utf-8")
        self.project.joinpath("CMakeLists.txt").write_text(
            cmake.replace(f'"{ROOT.as_posix()}/tests/generation/arm_probe.cpp"', 'main.cpp')
            .replace("MODULE all", "MODULE arm"), encoding="utf-8")
        self.run_command(["cmake", "--build", str(build), "--parallel", "4"])
        self.run_command([str(executable)])
        self.assertNotIn("class ChassisAssembly", (build / "generated/robot_config.hpp").read_text())


if __name__ == "__main__":
    unittest.main()
