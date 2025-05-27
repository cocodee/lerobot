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
                    "left_shoulder_roll_actuator": [1, 2, "POWER_FLOW_R86"],
                    "left_shoulder_yaw_actuator": [1, 3, "POWER_FLOW_R52"],
                    "left_elbow_pitch_actuator": [1, 4, "POWER_FLOW_R52"],
                    "left_elbow_yaw_actuator": [1, 5, "POWER_FLOW_R52"],
                    "left_wrist_front_actuator": [1, 6, "POWER_FLOW_L28"],
                    "left_wrist_back_actuator": [1, 7, "POWER_FLOW_L28"],
                    "left_claw_actuator": [1, 8, "OMNI_PICKER"],
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
        agibotx1.get_controller().disable_all_actuator()
        #ret = self.motor_bus.enable_all_actuator()
        #if ret:
        #    print("Enable Actuator Success")
        #else:
        #    print("Enable Actuator Failed")
        for i in range(60*100):
            read_data = self.motor_bus.read("position")
            print(f"data:{read_data.tolist()}")
            time.sleep(0.01)
        agibotx1.get_controller().disable_all_actuator()
        agibotx1.get_controller().stop()            

    def test_zero(self):
        agibotx1.create_dcu(self.mock_config.dcu_name,self.mock_config.ethercat_id)
        self.motor_bus = AgibotX1MotorsBus(self.mock_config)
        self.motor_bus.connect()
        ret = agibotx1.start_controller("enp2s0")
        if not ret:
            print("Start Failed") 
        self.motor_bus.enable_all_actuator()
        self.motor_bus.write("position",[0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5])
        time.sleep(20)
        agibotx1.get_controller().disable_all_actuator()
        agibotx1.get_controller().stop()  

if __name__ == "__main__":
    unittest.main()
