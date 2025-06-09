import unittest
from unittest.mock import patch
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
                    "right_shoulder_pitch_actuator": [1, 1, "POWER_FLOW_R86"],
                    "right_shoulder_roll_actuator": [1, 2, "POWER_FLOW_R86"],
                    "right_shoulder_yaw_actuator": [1, 3, "POWER_FLOW_R52"],
                    "right_elbow_pitch_actuator": [1, 4, "POWER_FLOW_R52"],
                    "right_elbow_yaw_actuator": [1, 5, "POWER_FLOW_R52"],
                    "right_wrist_front_actuator": [1, 6, "POWER_FLOW_L28"],
                    "right_wrist_back_actuator": [1, 7, "POWER_FLOW_L28"],
                    "right_claw_actuator": [1, 8, "OMNI_PICKER"],
            },
        )
        self.motor_bus = AgibotX1MotorsBus(self.mock_config)
    def test_enable(self):
        agibotx1.create_dcu(self.mock_config.dcu_name,self.mock_config.ethercat_id)
        self.motor_bus = AgibotX1MotorsBus(self.mock_config)
        self.motor_bus.connect()
        ret = agibotx1.start_controller("enp2s0")
        if not ret:
            print("Start Failed")        
        agibotx1.get_controller().enable_all_actuator()
        time.sleep(10)
        agibotx1.get_controller().disable_all_actuator()
        agibotx1.get_controller().stop()            

if __name__ == "__main__":
    unittest.main()
