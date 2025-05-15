import logging
from lerobot.common.robot_devices.motors.configs import AgibotX1MotorsBusConfig
from mock_xybercontrller import MockXyberController as XyberController
from mock_xybercontrller import MockActautorMode, MockActautorState, MockActuatorType, MockCtrlChannel as ActautorMode,ActautorState,ActuatorType,CtrlChannel
import numpy as np


CTRL_CHANNEL = 'ctrl_channel'
CAN_ID = 'can_id'
ACTURATOR_TYPE = 'actuator_type'
IF_NAME = 'if_name'

POWER_FLOW_R86 = "POWER_FLOW_R86"
POWER_FLOW_R52 = "POWER_FLOW_R52"
POWER_FLOW_L28 = "POWER_FLOW_L28"
OMNI_PICKER = "OMNI_PICKER"
'''
AgibotX1RobotConfig(
                dcu_name="body",
                ethercat_id=1,
                if_name="eth0",
                motors={
                    # name: (ctrl_channel,can_id,actuator_type)
                    "left_1": [1, 1, "POWER_FLOW_R86"],
                    "left_2": [1, 2, "POWER_FLOW_R86"],
                    "left_3": [1, 3, "POWER_FLOW_R52"],
                    "left_4": [1, 4, "POWER_FLOW_R52"],
                    "left_5": [1, 5, "POWER_FLOW_R52"],
                    "left_6": [1, 6, "POWER_FLOW_L28"],
                    "left_7": [1, 7, "POWER_FLOW_L28"],
                    "left_8": [1, 7, "OMNI_PICKER"],
                },
            ),
'''
class AgibotX1MotorsBus():
    def __init__(
        self,
        config: AgibotX1MotorsBusConfig,
    ):
        self.dcu_name = config.dcu_name
        self.ethercat_id = config.ethercat_id
        self.motors = config.motors
        self.if_name == config.if_name
        self.kp = 0.9
        self.kd = 0.2
        self.mock = config.mock
        self.calibration = None
        self.is_connected = False
        self.logs = {}

    def connect(self):
        self.controller = XyberController.get_instance()
        for motor_name,motor in self.motors:
            self.controller.create_dcu(self.dcu_name, self.ethercat_id)
            self.controller.attach_actuator(
                dcu_name=self.dcu_name,
                ch=motor[CTRL_CHANNEL],
                type=motor[ACTURATOR_TYPE],
                actuator_name=motor_name,
                can_id=motor[CAN_ID],
            )
        self.controller.set_realtime(rt_priority=90, bind_cpu=1)
        self.controller.start(ifname=self.if_name, cycle_ns=1000000, enable_dc=True)
        self.is_connected = True
    def reconnect(self):
        if self.is_connected:
            self.controller.stop()
        self.connect()
    def motor_names(self) -> list[str]:
        return list(self.motors.keys())

    def set_calibration(self, calibration: dict[str, list]):
        self.calibration = calibration
    def apply_calibration(self, values: np.ndarray | list, motor_names: list[str] | None):
        if not self.calibration:
            raise ValueError("Calibration data not set.")
        motor_names = motor_names or self.motor_names()
        calibrated_values = []
        for i, name in enumerate(motor_names):
            calib_data = self.calibration.get(name, [0, 1])  # Default no calibration
            calibrated_values.append((values[i] - calib_data[0]) / calib_data[1])
        return calibrated_values
    def revert_calibration(self, values: np.ndarray | list, motor_names: list[str] | None):
        if not self.calibration:
            raise ValueError("Calibration data not set.")
        motor_names = motor_names or self.motor_names()
        reverted_values = []
        for i, name in enumerate(motor_names):
            calib_data = self.calibration.get(name, [0, 1])  # Default no calibration
            reverted_values.append(values[i] * calib_data[1] + calib_data[0])
        return reverted_values
    def read(self, data_name, motor_names: str | list[str] | None = None):
        if not self.is_connected:
            raise ConnectionError("Not connected to the motors bus.")
        motor_names = motor_names or self.motor_names()
        if isinstance(motor_names, str):
            motor_names = [motor_names]
        values = []
        reader_map = {
            'position': self.controller.get_position,
            'velocity': self.controller.get_velocity,
            'effort': self.controller.get_effort,
        }
        reader = reader_map.get(data_name)
        if not reader:
            raise ValueError(f"Unknown data_name: {data_name}")
        for name in motor_names:
            values.append(reader(name))
        return np.array(values)

    def write(self, data_name, values: int | float | np.ndarray, motor_names: str | list[str] | None = None):
        if not self.is_connected:
            raise ConnectionError("Not connected to the motors bus.")
        motor_names = motor_names or self.motor_names()
        if isinstance(motor_names, str):
            motor_names = [motor_names]
        if not isinstance(values, (list, np.ndarray)):
            values = [values] * len(motor_names)
        if len(values) != len(motor_names):
            raise ValueError("Length of values must match the number of motor names.")
        if data_name == 'mit_cmd':
            for name, value in zip(motor_names, values):
                pos = value
                self.controller.set_mit_cmd(name, pos, 0, 0, self.kp, self.kd)
        else:
            raise ValueError(f"Unknown data_name: {data_name}")
