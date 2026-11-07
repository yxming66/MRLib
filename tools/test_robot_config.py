"""Regression checks for the v2 configuration contract (no generation/hardware)."""
from copy import deepcopy
from pathlib import Path
import unittest

import yaml

from tools.validate_robot_config import UniqueLoader, validate

ROOT = Path(__file__).resolve().parents[1]


class RobotConfigTests(unittest.TestCase):
    def setUp(self):
        self.config = yaml.load(
            (ROOT / "examples/h7_integration/config.yaml").read_text(encoding="utf-8"),
            Loader=UniqueLoader)

    def test_complete_example(self):
        validate(self.config)

    def test_other_project_with_chassis_only(self):
        self.config["project"]["name"] = "other_robot"
        del self.config["modules"]["arm"]
        self.config["devices"]["joints"] = {}
        for name in ("arm_0", "arm_1", "arm_2"):
            del self.config["devices"]["motors"][name]
        self.config["tasks"]["robot"]["owns_modules"] = ["chassis"]
        validate(self.config)

    def test_unknown_fields_and_unsupported_version(self):
        for mutation in ({"typo": 1}, {"version": 1}):
            config = deepcopy(self.config)
            config.update(mutation)
            with self.assertRaises(ValueError):
                validate(config)

    def test_duplicate_yaml_keys(self):
        with self.assertRaisesRegex(ValueError, "duplicate key"):
            yaml.load("version: 2\nversion: 1\n", Loader=UniqueLoader)

    def test_old_dm_address_conflicts_with_rm_feedback(self):
        self.config["devices"]["motors"]["arm_0"]["can_id"] = 1
        with self.assertRaisesRegex(ValueError, "CAN endpoint conflict"):
            validate(self.config)

    def test_missing_device_reference(self):
        self.config["devices"]["wheels"]["front_right"]["motor"] = "missing"
        with self.assertRaisesRegex(ValueError, "unknown reference"):
            validate(self.config)

    def test_inverted_limits_and_nonunit_axis(self):
        config = deepcopy(self.config)
        config["devices"]["joints"]["shoulder"]["limits"]["min_rad"] = 2
        with self.assertRaisesRegex(ValueError, "min_rad"):
            validate(config)
        self.config["modules"]["arm"]["joints"][0]["axis_in_joint_frame"] = [0, 0, 0]
        with self.assertRaisesRegex(ValueError, "unit vector"):
            validate(self.config)

    def test_multiple_task_owners(self):
        self.config["tasks"]["second"] = deepcopy(self.config["tasks"]["robot"])
        with self.assertRaisesRegex(ValueError, "task owner"):
            validate(self.config)

    def test_multiple_physical_port_aliases(self):
        self.config["board"]["bindings"]["can"]["other"] = "BSP_CAN_1"
        with self.assertRaisesRegex(ValueError, "physical endpoint"):
            validate(self.config)

    def test_nonfinite_values(self):
        self.config["devices"]["wheels"]["front_right"]["radius_m"] = float("nan")
        with self.assertRaisesRegex(ValueError, "finite"):
            validate(self.config)

    def test_uart_endpoints_reject_duplicate_aliases(self):
        self.config["board"]["bindings"]["uart"] = {"remote": "uart9", "telemetry_uart": "uart9"}
        with self.assertRaisesRegex(ValueError, "physical endpoint"):
            validate(self.config)

    def test_named_custom_business_parameters(self):
        self.config["extensions"]["team_name"] = {"mode": "hold", "threshold": 0.2}
        validate(self.config)


if __name__ == "__main__":
    unittest.main()
