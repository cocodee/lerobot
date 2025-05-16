import logging
from lerobot.common.robot_devices.motors.configs import AgibotX1MotorsBusConfig
from mock_xybercontrller import MockXyberController as XyberController
from mock_xybercontrller import MockActautorMode, MockActautorState, MockActuatorType, MockCtrlChannel as ActautorMode,ActautorState,ActuatorType,CtrlChannel
import numpy as np
import math
from copy import deepcopy


CTRL_CHANNEL = 'ctrl_channel'
CAN_ID = 'can_id'
ACTURATOR_TYPE = 'actuator_type'
IF_NAME = 'if_name'

CTRL_CHANNEL_INDEX = 0
CAN_ID_INDEX = 1
ACTURATOR_TYPE_INDEX = 2

POWER_FLOW_R86 = "POWER_FLOW_R86"
POWER_FLOW_R52 = "POWER_FLOW_R52"
POWER_FLOW_L28 = "POWER_FLOW_L28"
OMNI_PICKER = "OMNI_PICKER"

DATA_NAMES = ["position", "velocity", "effort"]
DATA_NAME_POSITION = "position"
DATA_NAME_VELOCITY = "velocity"
DATA_NAME_EFFORT = "effort"

HALF_TURN_DEGREE=180
LOWER_BOUND_DEGREE = -270
UPPER_BOUND_DEGREE = 270
LOWER_BOUND_LINEAR = -10
UPPER_BOUND_LINEAR = 110

ACTURATOR_TYPE_RESOLUTION = {
    "POWER_FLOW_R86": math.pi*2,
    "POWER_FLOW_R52": math.pi*2,
    "POWER_FLOW_L28": math.pi*2,
    "OMNI_PICKER": 1,
}
'''
  - left_shoulder_pitch_actuator
  - left_shoulder_roll_actuator
  - left_shoulder_yaw_actuator
  - left_elbow_pitch_actuator
  - left_elbow_yaw_actuator
  - left_wrist_front_actuator
  - left_wrist_back_actuator
  - left_claw_actuator
AgibotX1RobotConfig(
                dcu_name="body",
                ethercat_id=1,
                if_name="eth0",
                motors={
                    # name: (ctrl_channel,can_id,actuator_type)
                    "left_shoulder_pitch_actuator": [1, 1, "POWER_FLOW_R86"],
                    "left_shoulder_roll_actuator": [1, 2, "POWER_FLOW_R86"],
                    "left_shoulder_yaw_actuator": [1, 3, "POWER_FLOW_R52"],
                    "left_elbow_pitch_actuator": [1, 4, "POWER_FLOW_R52"],
                    "left_elbow_yaw_actuator": [1, 5, "POWER_FLOW_R52"],
                    "left_wrist_front_actuator": [1, 6, "POWER_FLOW_L28"],
                    "left_wrist_back_actuator": [1, 7, "POWER_FLOW_L28"],
                    "left_claw_actuator": [1, 7, "OMNI_PICKER"],
                },
            ),

'''


class CalibrationMode(enum.Enum):
    # Joints with rotational motions are expressed in degrees in nominal range of [-180, 180]
    DEGREE = 0
    # Joints with linear motions (like gripper of Aloha) are expressed in nominal range of [0, 100]
    LINEAR = 1

class JointOutOfRangeError(Exception):
    def __init__(self, message="Joint is out of range"):
        self.message = message
        super().__init__(self.message)

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
        self.acturator_type_resolution = deepcopy(ACTURATOR_TYPE_RESOLUTION)

    def connect(self):
        self.controller = XyberController.get_instance()
        for motor_name,motor in self.motors:
            self.controller.create_dcu(self.dcu_name, self.ethercat_id)
            self.controller.attach_actuator(
                dcu_name=self.dcu_name,
                ch=motor[CTRL_CHANNEL_INDEX],
                type=motor[ACTURATOR_TYPE_INDEX],
                actuator_name=motor_name,
                can_id=motor[CAN_ID_INDEX],
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

        motor_names = motor_names or self.motor_names()
        calibrated_values = []
        for i, name in enumerate(motor_names):
            calib_data = self.calibration.get(name, [0, 1])  # Default no calibration
            calibrated_values.append((values[i] - calib_data[0]) / calib_data[1])
    
        if not self.calibration:
            raise ValueError("Calibration data not set.")    
        if motor_names is None:
                motor_names = self.motor_names

        # Convert from unsigned int32 original range [0, 2**32] to signed float32 range
        values = values.astype(np.float32)

        for i, name in enumerate(motor_names):
            calib_idx = self.calibration["motor_names"].index(name)
            calib_mode = self.calibration["calib_mode"][calib_idx]

            if CalibrationMode[calib_mode] == CalibrationMode.DEGREE:
                drive_mode = self.calibration["drive_mode"][calib_idx]
                homing_offset = self.calibration["homing_offset"][calib_idx]
                _,_,acturator_type = self.motors[name]
                resolution = self.acturator_type_resolution[acturator_type]

                # Update direction of rotation of the motor to match between leader and follower.
                # In fact, the motor of the leader for a given joint can be assembled in an
                # opposite direction in term of rotation than the motor of the follower on the same joint.
                if drive_mode:
                    values[i] *= -1

                # Convert from range [-2**31, 2**31] to
                # nominal range [-resolution//2, resolution//2] (e.g. [-2048, 2048])
                values[i] += homing_offset

                # Convert from range [-resolution//2, resolution//2] to
                # universal float32 centered degree range [-180, 180]
                # (e.g. 2048 / (4096 // 2) * 180 = 180)
                values[i] = values[i] / (resolution // 2) * HALF_TURN_DEGREE

                if (values[i] < LOWER_BOUND_DEGREE) or (values[i] > UPPER_BOUND_DEGREE):
                    raise JointOutOfRangeError(
                        f"Wrong motor position range detected for {name}. "
                        f"Expected to be in nominal range of [-{HALF_TURN_DEGREE}, {HALF_TURN_DEGREE}] degrees (a full rotation), "
                        f"with a maximum range of [{LOWER_BOUND_DEGREE}, {UPPER_BOUND_DEGREE}] degrees to account for joints that can rotate a bit more, "
                        f"but present value is {values[i]} degree. "
                        "This might be due to a cable connection issue creating an artificial 360 degrees jump in motor values. "
                        "You need to recalibrate by running: `python lerobot/scripts/control_robot.py calibrate`"
                    )

            elif CalibrationMode[calib_mode] == CalibrationMode.LINEAR:
                start_pos = self.calibration["start_pos"][calib_idx]
                end_pos = self.calibration["end_pos"][calib_idx]

                # Rescale the present position to a nominal range [0, 100] %,
                # useful for joints with linear motions like Aloha gripper
                values[i] = (values[i] - start_pos) / (end_pos - start_pos) * 100

                if (values[i] < LOWER_BOUND_LINEAR) or (values[i] > UPPER_BOUND_LINEAR):
                    raise JointOutOfRangeError(
                        f"Wrong motor position range detected for {name}. "
                        f"Expected to be in nominal range of [0, 100] % (a full linear translation), "
                        f"with a maximum range of [{LOWER_BOUND_LINEAR}, {UPPER_BOUND_LINEAR}] % to account for some imprecision during calibration, "
                        f"but present value is {values[i]} %. "
                        "This might be due to a cable connection issue creating an artificial jump in motor values. "
                        "You need to recalibrate by running: `python lerobot/scripts/control_robot.py calibrate`"
                    )

        return values
    def revert_calibration(self, values: np.ndarray | list, motor_names: list[str] | None):
        if not self.calibration:
            raise ValueError("Calibration data not set.")
        motor_names = motor_names or self.motor_names()

        # Convert from signed float32 range back to original range [0, 2**32]
        values = values.astype(np.float32)

        for i, name in enumerate(motor_names):
            calib_idx = self.calibration["motor_names"].index(name)
            calib_mode = self.calibration["calib_mode"][calib_idx]

            if CalibrationMode[calib_mode] == CalibrationMode.DEGREE:
                drive_mode = self.calibration["drive_mode"][calib_idx]
                homing_offset = self.calibration["homing_offset"][calib_idx]
                _,_,acturator_type = self.motors[name]
                resolution = self.acturator_type_resolution[acturator_type]

                # Reverse universal float32 centered degree range [-180, 180] to
                # range [-resolution//2, resolution//2] (e.g. [-2048, 2048])
                values[i] = values[i] / HALF_TURN_DEGREE * (resolution // 2)

                # Reverse nominal range [-resolution//2, resolution//2] to
                # range [-2**31, 2**31]
                values[i] -= homing_offset

                # Reverse direction of rotation if necessary
                if drive_mode:
                    values[i] *= -1

            elif CalibrationMode[calib_mode] == CalibrationMode.LINEAR:
                start_pos = self.calibration["start_pos"][calib_idx]
                end_pos = self.calibration["end_pos"][calib_idx]

                # Reverse rescale from nominal range [0, 100] % to original position range
                values[i] = values[i] / 100 * (end_pos - start_pos) + start_pos

        return values
    def read(self, data_name, motor_names: str | list[str] | None = None):
        if not self.is_connected:
            raise ConnectionError("Not connected to the motors bus.")
        motor_names = motor_names or self.motor_names()
        if isinstance(motor_names, str):
            motor_names = [motor_names]
        values = []
        reader_map = {
            DATA_NAME_POSITION: self.controller.get_position,
            DATA_NAME_VELOCITY: self.controller.get_velocity,
            DATA_NAME_EFFORT: self.controller.get_effort,
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
        for name, value in zip(motor_names, values):
            if data_name == DATA_NAME_POSITION:
                pos = value
                self.controller.set_mit_cmd(name, pos, 0, 0, self.kp, self.kd)
            elif data_name == DATA_NAME_VELOCITY:
                vel = value
                self.controller.set_mit_cmd(name, 0, vel, 0, self.kp, self.kd)
            elif data_name == DATA_NAME_EFFORT:
                eff = value
                self.controller.set_mit_cmd(name, 0, 0, eff, self.kp, self.kd)
            else:
                raise ValueError(f"Unknown data_name: {data_name}")
