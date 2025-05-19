import unittest
from unittest.mock import MagicMock, patch
import numpy as np
from lerobot.common.robot_devices.motors.utils import MotorsBus
from lerobot.common.robot_devices.motors.agibotx1 import (
    AgibotX1MotorsBus,
    AgibotX1MotorsBusConfig,
    CalibrationMode,
    JointOutOfRangeError,
)

class TestAgibotX1MotorsBus(unittest.TestCase):
    def setUp(self):
        # Mock configuration
        self.mock_config = AgibotX1MotorsBusConfig(
            dcu_name="test_dcu",
            ethercat_id=1,
            if_name="eth0",
            motors={
                "test_motor": [1, 1, "POWER_FLOW_R86"],
            },
            mock=True,
        )
        self.motor_bus = AgibotX1MotorsBus(self.mock_config)

    @patch("lerobot.common.robot_devices.motors.agibotx1.XyberController.get_instance")
    def test_connect(self, mock_get_instance):
        mock_controller = MagicMock()
        mock_get_instance.return_value = mock_controller

        self.motor_bus.connect()

        mock_controller.attach_actuator.assert_called_once_with(
            dcu_name="test_dcu",
            ch=1,
            type="POWER_FLOW_R86",
            actuator_name="test_motor",
            can_id=1,
        )
        self.assertTrue(self.motor_bus.is_connected)

    def test_set_calibration(self):
        calibration_data = {"test_motor": [10, 2]}
        self.motor_bus.set_calibration(calibration_data)

        self.assertEqual(self.motor_bus.calibration, calibration_data)

    def test_apply_calibration_degree_mode(self):
        self.motor_bus.calibration = {
            "motor_names": ["test_motor"],
            "calib_mode": ["DEGREE"],
            "drive_mode": [0],
            "homing_offset": [0],
        }
        values = np.array([100])
        calibrated_values = self.motor_bus.apply_calibration(values, ["test_motor"])

        # Expected transformation based on POWER_FLOW_R86 resolution (2π)
        expected_value = 100 / (2 * np.pi // 2) * 180
        self.assertAlmostEqual(calibrated_values[0], expected_value)

    def test_apply_calibration_invalid_range(self):
        self.motor_bus.calibration = {
            "motor_names": ["test_motor"],
            "calib_mode": ["DEGREE"],
            "drive_mode": [0],
            "homing_offset": [0],
        }
        values = np.array([1000])  # Out of range value

        with self.assertRaises(JointOutOfRangeError):
            self.motor_bus.apply_calibration(values, ["test_motor"])

    def test_revert_calibration(self):
        self.motor_bus.calibration = {
            "motor_names": ["test_motor"],
            "calib_mode": ["DEGREE"],
            "drive_mode": [0],
            "homing_offset": [0],
        }
        calibrated_values = np.array([90])  # 90 degrees
        original_values = self.motor_bus.revert_calibration(calibrated_values, ["test_motor"])

        # Reverse transformation
        expected_value = (90 / 180) * (2 * np.pi // 2)
        self.assertAlmostEqual(original_values[0], expected_value)

    def test_read_without_connection(self):
        with self.assertRaises(ConnectionError):
            self.motor_bus.read("position")

    @patch("lerobot.common.robot_devices.motors.agibotx1.XyberController.get_instance")
    def test_write_without_connection(self, mock_get_instance):
        mock_controller = MagicMock()
        mock_get_instance.return_value = mock_controller

        with self.assertRaises(ConnectionError):
            self.motor_bus.write("position", 50)

if __name__ == "__main__":
    unittest.main()
