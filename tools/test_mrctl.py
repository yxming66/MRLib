"""End-to-end tests for the project generator and target dependency graph."""

from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
MRCTL = ROOT / "tools" / "mrctl.py"


class ProjectCliTests(unittest.TestCase):
    def setUp(self) -> None:
        self.test_workspace = ROOT / "build" / "mrctl-tests" / self._testMethodName
        if self.test_workspace.exists():
            shutil.rmtree(self.test_workspace)
        self.test_workspace.mkdir(parents=True)
        self.addCleanup(shutil.rmtree, self.test_workspace, True)
        self.project = self.test_workspace / "robot"
        self.library = Path(os.path.relpath(ROOT, self.project)).as_posix()
        build_root = Path(
            os.environ.get("MR_TESTLIB_TEST_BUILD_ROOT", ROOT / "build" / "mrctl-builds")
        )
        # Keep generated object paths short enough for Windows toolchains.
        test_id = {
            "test_host_project_compiles_only_selected_public_headers": "headers",
            "test_init_never_overwrites_existing_application_files": "init",
            "test_only_selected_target_dependencies_are_built": "deps",
            "test_target_schema_matches_the_cli": "schema",
            "test_unknown_targets_are_rejected": "unknown",
        }[self._testMethodName]
        self.build = build_root.resolve() / test_id
        if self.build.exists():
            shutil.rmtree(self.build)
        self.build.mkdir(parents=True)
        self.addCleanup(shutil.rmtree, self.build, True)

    def configure_project(self) -> subprocess.CompletedProcess[str]:
        return self.invoke("configure", str(self.project), "--build-dir", str(self.build))

    def build_project(self) -> subprocess.CompletedProcess[str]:
        return self.invoke("build", str(self.project), "--build-dir", str(self.build))

    def invoke(self, *arguments: str, check: bool = True) -> subprocess.CompletedProcess[str]:
        result = subprocess.run(
            [sys.executable, str(MRCTL), *arguments],
            cwd=ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        if check and result.returncode != 0:
            self.fail(f"command failed ({result.returncode}):\n{result.stdout}")
        return result

    def init(self) -> None:
        self.invoke(
            "init",
            str(self.project),
            "--name",
            "robot_test",
            "--library",
            self.library,
            "--targets",
            "mr_robotics_arm,mr_module_chassis",
        )

    def test_host_project_compiles_only_selected_public_headers(self) -> None:
        self.init()
        configured = self.configure_project()
        self.assertIn("resolved dependencies:", configured.stdout)
        self.assertIn("mr_component_math", configured.stdout)
        probe = self.build / "mrctl_compile_check.cpp"
        selected_headers = probe.read_text(encoding="utf-8")
        self.assertIn('#include "robotics/arm/arm.hpp"', selected_headers)
        self.assertIn('#include "module/chassis/chassis.hpp"', selected_headers)
        self.assertIn('#include "component/filter/filter.hpp"', selected_headers)
        self.assertIn('#include "component/math/scalar.hpp"', selected_headers)
        self.build_project()

        # Changing selection updates the build-only probe without touching app files.
        (self.project / "project.yaml").write_text(
            "version: 1\nproject:\n"
            f"  name: robot_test\n  library: {self.library}\n"
            "  targets:\n    - mr_component_filter\n",
            encoding="utf-8",
        )
        self.configure_project()
        self.assertEqual(
            probe.read_text(encoding="utf-8"),
            '#include "component/math/scalar.hpp"\n'
            '#include "component/filter/filter.hpp"\n'
            "void mr_project_selected_headers_compile_check() {}\n",
        )
        self.build_project()
        cache = (self.build / "CMakeCache.txt").read_text(encoding="utf-8")
        self.assertIn("MR_TESTLIB_PROJECT_TARGETS:STRING=mr::component_filter", cache)
        self.assertTrue((self.project / "src" / "main.cpp").is_file())

    def test_only_selected_target_dependencies_are_built(self) -> None:
        self.invoke(
            "init",
            str(self.project),
            "--name",
            "robot_filter_only",
            "--library",
            self.library,
            "--targets",
            "mr_component_filter",
        )
        self.configure_project()
        probe = (self.build / "mrctl_compile_check.cpp").read_text(encoding="utf-8")
        self.assertIn('#include "component/math/scalar.hpp"', probe)
        self.assertIn('#include "component/filter/filter.hpp"', probe)
        self.build_project()
        self.assertFalse(list(self.build.rglob("libmr_robotics_core.a")))
        self.assertFalse(list(self.build.rglob("mr_robotics_core.lib")))

    def test_target_schema_matches_the_cli(self) -> None:
        import importlib.util

        specification = importlib.util.spec_from_file_location("mrctl", MRCTL)
        self.assertIsNotNone(specification)
        assert specification.loader is not None
        module = importlib.util.module_from_spec(specification)
        specification.loader.exec_module(module)
        schema = json.loads((ROOT / "docs" / "project.schema.json").read_text())
        enum = schema["properties"]["project"]["properties"]["targets"]["items"]["enum"]
        self.assertEqual(set(enum), set(module.TARGET_CATALOG))
        self.assertEqual(
            module.resolve_targets(["mr_module_chassis"]),
            [
                "mr_component_math",
                "mr_robotics_chassis",
                "mr_component_filter",
                "mr_component_controller",
                "mr_device_wheel_interface",
                "mr_module_chassis",
            ],
        )

    def test_init_never_overwrites_existing_application_files(self) -> None:
        (self.project / "src").mkdir(parents=True)
        app = self.project / "src" / "main.cpp"
        app.write_text("// hand-written application\n", encoding="utf-8")
        self.init()
        self.assertEqual(app.read_text(encoding="utf-8"), "// hand-written application\n")
        self.assertTrue((self.project / "project.yaml").is_file())
        self.init()
        self.assertEqual(app.read_text(encoding="utf-8"), "// hand-written application\n")

    def test_unknown_targets_are_rejected(self) -> None:
        result = self.invoke(
            "init",
            str(self.project),
            "--name",
            "robot_test",
            "--library",
            self.library,
            "--targets",
            "robotics_does_not_exist",
            check=False,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse((self.project / "CMakeLists.txt").exists())


if __name__ == "__main__":
    unittest.main(verbosity=2)
