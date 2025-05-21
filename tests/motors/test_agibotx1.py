import unittest
from unittest.mock import patch
import numpy as np
from lerobot.common.robot_devices.motors.utils import MotorsBus
from lerobot.common.robot_devices.motors.agibotx1 import (
    AgibotX1MotorsBus,
    AgibotX1MotorsBusConfig,
    CalibrationMode,
    JointOutOfRangeError,
)
import lerobot.common.robot_devices.motors.agibotx1 as agibotx1
import time

class TestAgibotX1MotorsBus(unittest.TestCase):
    def setUp(self):
        # Mock configuration
        self.mock_config = AgibotX1MotorsBusConfig(
            dcu_name="body",
            ethercat_id=1,
            motors={
                    # name: (ctrl_channel,can_id,actuator_type)
                "left_shoulder_pitch_actuator": [1, 1, "POWER_FLOW_R86"],
            },
        )
        self.motor_bus = AgibotX1MotorsBus(self.mock_config)
    def test_connect_and_read(self):
        agibotx1.create_dcu(self.mock_config.dcu_name,self.mock_config.ethercat_id)
        self.motor_bus = AgibotX1MotorsBus(self.mock_config)

        self.motor_bus.connect()
        ret = agibotx1.start_controller("enp2s0")
        if not ret:
            print("Start Failed")
        ret = agibotx1.get_controller().enable_all_actuator()
        if ret:
            print("Enable Actuator Success")
        else:
            print("Enable Actuator Failed")
        read_data = self.motor_bus.read("position")
        self.assertTrue(self.motor_bus.is_connected)
        self.assertIsInstance(read_data, np.ndarray)
        time.sleep(5)
        print(f"data:{read_data.tolist()}")
        # add 1 to read_data
        print(f"write data:{(read_data+2).tolist()}")
        self.motor_bus.write("position", read_data + 2)
        time.sleep(10)
        self.motor_bus.disconnect()

if __name__ == "__main__":
    unittest.main()
