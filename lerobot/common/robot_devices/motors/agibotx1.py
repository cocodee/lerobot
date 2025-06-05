import logging
from lerobot.common.robot_devices.motors.configs import AgibotX1MotorsBusConfig
#from lerobot.common.robot_devices.motors.mock_xybercontroller import MockXyberController as XyberController
#from lerobot.common.robot_devices.motors.mock_xybercontroller import(
#    MockActautorMode as ActautorMode,        # MockActautorMode 重命名为 ActautorMode
#    MockActautorState as ActautorState,      # MockActautorState 重命名为 ActautorState
#    MockActuatorType as ActuatorType,        # MockActuatorType 重命名为 ActuatorType
#    MockCtrlChannel as CtrlChannel          # MockCtrlChannel 重命名为 CtrlChannel
#)
from lerobot.common.robot_devices.motors.xyber_controller_py import XyberController as XyberController
from lerobot.common.robot_devices.motors.xyber_controller_py import(
    ActautorMode as ActautorMode,        # MockActautorMode 重命名为 ActautorMode
    ActautorState as ActautorState,      # MockActautorState 重命名为 ActautorState
    ActuatorType as ActuatorType,        # MockActuatorType 重命名为 ActuatorType
    CtrlChannel as CtrlChannel          # MockCtrlChannel 重命名为 CtrlChannel
)
from lerobot.common.robot_devices.utils import RobotDeviceAlreadyConnectedError, RobotDeviceNotConnectedError
import numpy as np
import math
import enum
import time
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

'''
https://github.com/AgibotTech/agibot_x1_infer/blob/main/doc/dcu_driver_module/dcu_driver_module.zh_CN.md
'''
ACTURATOR_TYPE_RESOLUTION = {
    "POWER_FLOW_R86": math.pi*4,
    "POWER_FLOW_R52": math.pi*4,
    "POWER_FLOW_L28": 10,
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

CALIBRATION_REQUIRED = ["position"]


class CalibrationMode(enum.Enum):
    # Joints with rotational motions are expressed in degrees in nominal range of [-180, 180]
    DEGREE = 0
    # Joints with linear motions (like gripper of Aloha) are expressed in nominal range of [0, 100]
    LINEAR = 1

class JointOutOfRangeError(Exception):
    def __init__(self, message="Joint is out of range"):
        self.message = message
        super().__init__(self.message)

def get_controller():
    return XyberController.get_instance()
def start_controller(if_name:str):
    controller = XyberController.get_instance()
    controller.set_realtime(rt_priority=90, bind_cpu=1)
    return controller.start(ifname=if_name, cycle_ns=1000000, enable_dc=True)
def create_dcu(dcu_name, ethercat_id):
    controller = XyberController.get_instance()
    return controller.create_dcu(dcu_name, ethercat_id)

def stop_controller():
    controller = XyberController.get_instance()
    controller.disable_all_actuator()
    return controller.stop()

def to_dcu_channel(ch:int):
      if ch == 1:
          return CtrlChannel.CTRL_CH1
      elif ch == 2:
          return CtrlChannel.CTRL_CH2
      elif ch == 3:
          return CtrlChannel.CTRL_CH3
      else:
          raise ValueError("Invalid channel")
def to_dcu_actuator_type(actuator_type:str):
      if actuator_type == "POWER_FLOW_R86":
          return ActuatorType.POWER_FLOW_R86
      elif  actuator_type == "POWER_FLOW_R52":
          return ActuatorType.POWER_FLOW_R52
      elif actuator_type == "POWER_FLOW_L28":
          return ActuatorType.POWER_FLOW_L28
      elif actuator_type == "OMNI_PICKER":
          return ActuatorType.OMNI_PICKER
      else:
          raise ValueError("Invalid actuator type")
class AgibotX1MotorsBus():
    def __init__(
        self,
        config: AgibotX1MotorsBusConfig,
    ):
        self.dcu_name = config.dcu_name
        self.ethercat_id = config.ethercat_id
        self.motors = config.motors
        self.kp = 15
        self.kd = 1
        self.mock = config.mock
        self.calibration = None
        self.is_connected = False
        self.logs = {}
        self.acturator_type_resolution = deepcopy(ACTURATOR_TYPE_RESOLUTION)
        self.controller = get_controller()
        self.enabled = False

    def connect(self):
        for motor_name, motor in self.motors.items():
            ret = self.controller.attach_actuator(
                dcu_name=self.dcu_name,
                ch=to_dcu_channel(motor[CTRL_CHANNEL_INDEX]),
                type=to_dcu_actuator_type(motor[ACTURATOR_TYPE_INDEX]),
                actuator_name=motor_name,
                can_id=motor[CAN_ID_INDEX],
            )
            if not ret:
                print(f"Failed to attach actuator {motor_name}")
        self.is_connected = True
    def reconnect(self):
        if self.is_connected:
            self.controller.stop()
        self.connect()

    def disconnect(self):
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                f"DynamixelMotorsBus({self.port}) is not connected. Try running `motors_bus.connect()` first."
            )
        self.disable_all_actuator()
        self.is_connected = False        
    def motor_names(self) -> list[str]:
        return list(self.motors.keys())

    def set_calibration(self, calibration: dict[str, list]):
        self.calibration = calibration

    def apply_calibration_autocorrect(self, values: np.ndarray | list, motor_names: list[str] | None):
        """This function applies the calibration, automatically detects out of range errors for motors values and attempts to correct.

        For more info, see docstring of `apply_calibration` and `autocorrect_calibration`.
        """
        try:
            values = self.apply_calibration(values, motor_names)
        except JointOutOfRangeError as e:
            print(e)
            self.autocorrect_calibration(values, motor_names)
            values = self.apply_calibration(values, motor_names)
        return values

    def autocorrect_calibration(self, values: np.ndarray | list, motor_names: list[str] | None):
        """This function automatically detects issues with values of motors after calibration, and correct for these issues.

        Some motors might have values outside of expected maximum bounds after calibration.
        For instance, for a joint in degree, its value can be outside [-270, 270] degrees, which is totally unexpected given
        a nominal range of [-180, 180] degrees, which represents half a turn to the left or right starting from zero position.

        Known issues:
        #1: Motor value randomly shifts of a full turn, caused by hardware/connection errors.
        #2: Motor internal homing offset is shifted by a full turn, caused by using default calibration (e.g Aloha).
        #3: motor internal homing offset is shifted by less or more than a full turn, caused by using default calibration
            or by human error during manual calibration.

        Issues #1 and #2 can be solved by shifting the calibration homing offset by a full turn.
        Issue #3 will be visually detected by user and potentially captured by the safety feature `max_relative_target`,
        that will slow down the motor, raise an error asking to recalibrate. Manual recalibrating will solve the issue.

        Note: A full turn corresponds to 360 degrees but also to 4096 steps for a motor resolution of 4096.
        """
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

                # Convert from initial range to range [-180, 180] degrees
                calib_val = (values[i] + homing_offset) / (resolution / 2) * HALF_TURN_DEGREE
                in_range = (calib_val > LOWER_BOUND_DEGREE) and (calib_val < UPPER_BOUND_DEGREE)

                # Solve this inequality to find the factor to shift the range into [-180, 180] degrees
                # values[i] = (values[i] + homing_offset + resolution * factor) / (resolution // 2) * HALF_TURN_DEGREE
                # - HALF_TURN_DEGREE <= (values[i] + homing_offset + resolution * factor) / (resolution // 2) * HALF_TURN_DEGREE <= HALF_TURN_DEGREE
                # (- (resolution // 2) - values[i] - homing_offset) / resolution <= factor <= ((resolution // 2) - values[i] - homing_offset) / resolution
                low_factor = (-(resolution / 2) - values[i] - homing_offset) / resolution
                upp_factor = ((resolution / 2) - values[i] - homing_offset) / resolution

            elif CalibrationMode[calib_mode] == CalibrationMode.LINEAR:
                start_pos = self.calibration["start_pos"][calib_idx]
                end_pos = self.calibration["end_pos"][calib_idx]

                # Convert from initial range to range [0, 100] in %
                calib_val = (values[i] - start_pos) / (end_pos - start_pos) * 100
                in_range = (calib_val > LOWER_BOUND_LINEAR) and (calib_val < UPPER_BOUND_LINEAR)

                # Solve this inequality to find the factor to shift the range into [0, 100] %
                # values[i] = (values[i] - start_pos + resolution * factor) / (end_pos + resolution * factor - start_pos - resolution * factor) * 100
                # values[i] = (values[i] - start_pos + resolution * factor) / (end_pos - start_pos) * 100
                # 0 <= (values[i] - start_pos + resolution * factor) / (end_pos - start_pos) * 100 <= 100
                # (start_pos - values[i]) / resolution <= factor <= (end_pos - values[i]) / resolution
                low_factor = (start_pos - values[i]) / resolution
                upp_factor = (end_pos - values[i]) / resolution

            if not in_range:
                # Get first integer between the two bounds
                if low_factor < upp_factor:
                    factor = math.ceil(low_factor)

                    if factor > upp_factor:
                        raise ValueError(f"No integer found between bounds [{low_factor=}, {upp_factor=}]")
                else:
                    factor = math.ceil(upp_factor)

                    if factor > low_factor:
                        raise ValueError(f"No integer found between bounds [{low_factor=}, {upp_factor=}]")

                if CalibrationMode[calib_mode] == CalibrationMode.DEGREE:
                    out_of_range_str = f"{LOWER_BOUND_DEGREE} < {calib_val} < {UPPER_BOUND_DEGREE} degrees"
                    in_range_str = f"{LOWER_BOUND_DEGREE} < {calib_val} < {UPPER_BOUND_DEGREE} degrees"
                elif CalibrationMode[calib_mode] == CalibrationMode.LINEAR:
                    out_of_range_str = f"{LOWER_BOUND_LINEAR} < {calib_val} < {UPPER_BOUND_LINEAR} %"
                    in_range_str = f"{LOWER_BOUND_LINEAR} < {calib_val} < {UPPER_BOUND_LINEAR} %"

                logging.warning(
                    f"Auto-correct calibration of motor '{name}' by shifting value by {abs(factor)} full turns, "
                    f"from '{out_of_range_str}' to '{in_range_str}'."
                )

                # A full turn corresponds to 360 degrees but also to 4096 steps for a motor resolution of 4096.
                self.calibration["homing_offset"][calib_idx] += resolution * factor        
    def apply_calibration(self, values: np.ndarray | list, motor_names: list[str] | None):
        #print(f"Applying calibration to {motor_names} {values}")
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
                values[i] = values[i] / (resolution / 2) * HALF_TURN_DEGREE

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
                values[i] = values[i] / HALF_TURN_DEGREE * (resolution / 2)

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
        if self.enabled == False:
            self.disable_all_actuator()
        for name in motor_names:
            v= self.controller.get_position(name)
            print(f"Reading {data_name} for {name} reader_value {v}")
            values.append(v)
        values = np.array(values)
        print(f"loaded {data_name} for {motor_names} {values}")
        if data_name in CALIBRATION_REQUIRED and self.calibration is not None:
            values = self.apply_calibration(values, motor_names)    
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
        values = np.array(values)
        if data_name in CALIBRATION_REQUIRED and self.calibration is not None:
            values = self.revert_calibration(values, motor_names)        
        print(f"Setting {data_name} to {values}")
        for name, value in zip(motor_names, values):
            if data_name == DATA_NAME_POSITION:
                pos = value
                cur = 0
                if name.endswith("claw_actuator"):
                    cur = 0.6
                else:
                    cur = 50
                ret = self.controller.set_mit_cmd(name, pos, 0, cur, self.kp, self.kd)
                logging.warning(f"Set {name} to {pos},result {ret}")
            elif data_name == DATA_NAME_VELOCITY:
                vel = value
                self.controller.set_mit_cmd(name, 0, vel, 0, self.kp, self.kd)
            elif data_name == DATA_NAME_EFFORT:
                eff = value
                self.controller.set_mit_cmd(name, 0, 0, eff, self.kp, self.kd)
            else:
                raise ValueError(f"Unknown data_name: {data_name}")
            
    def enable_all_actuator(self):
        for name in self.motor_names():
            ret = self.controller.enable_actuator(name)
            if not ret:
                print(f"Failed to enable actuator {name}")
        self.enabled = True
    
    def disable_all_actuator(self):
        for name in self.motor_names():
            ret = self.controller.disable_actuator(name)
            if not ret:
                print(f"Failed to disable actuator {name}")
        self.enabled = False
    def show_status(self,name:str):
        mode = self.controller.get_mode(name)
        pos = self.controller.get_position(name)
        vel = self.controller.get_velocity(name)
        toq = self.controller.get_effort(name)
        print(f"{name}: mode {mode}, pos {pos}, vel {vel}, toq {toq}")

        
