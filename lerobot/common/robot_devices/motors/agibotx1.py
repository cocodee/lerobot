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

# Default values for XyberController
DEFAULT_RT_PRIORITY = 90
DEFAULT_BIND_CPU = 1
DEFAULT_CYCLE_NS = 1000000  # 1ms

# Default PID gains for AgibotX1MotorsBus
DEFAULT_KP_GAIN = 0.9
DEFAULT_KD_GAIN = 0.2

# Calibration and value conversion constants
PERCENTAGE_CONVERSION_FACTOR = 100.0  # Using float for consistency
RESOLUTION_HALF_DIVISOR = 2.0     # Using float for consistency

# Default values for MIT command parameters
MIT_CMD_DEFAULT_POS = 0.0
MIT_CMD_DEFAULT_VEL = 0.0
MIT_CMD_DEFAULT_EFF = 0.0

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
    controller.set_realtime(rt_priority=DEFAULT_RT_PRIORITY, bind_cpu=DEFAULT_BIND_CPU)
    return controller.start(ifname=if_name, cycle_ns=DEFAULT_CYCLE_NS, enable_dc=True)
def create_dcu(dcu_name, ethercat_id):
    controller = XyberController.get_instance()
    return controller.create_dcu(dcu_name, ethercat_id)

def stop_controller():
    controller = XyberController.get_instance()
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
        self.kp = DEFAULT_KP_GAIN
        self.kd = DEFAULT_KD_GAIN
        self.mock = config.mock
        self.calibration = None
        self.is_connected = False
        self.logs = {}
        self.acturator_type_resolution = deepcopy(ACTURATOR_TYPE_RESOLUTION)
        self.controller = get_controller()

    def connect(self):
        if self.is_connected:
            raise RobotDeviceAlreadyConnectedError(
                f"AgibotX1MotorsBus for DCU '{self.dcu_name}' is already connected."
            )
        for motor_name, motor in self.motors.items():
            ret = self.controller.attach_actuator(
                dcu_name=self.dcu_name,
                ch=to_dcu_channel(motor[CTRL_CHANNEL_INDEX]),
                type=to_dcu_actuator_type(motor[ACTURATOR_TYPE_INDEX]),
                actuator_name=motor_name,
                can_id=motor[CAN_ID_INDEX],
            )
            if not ret: # Assuming False or a non-zero error code indicates failure
                # Attempt to detach any already attached actuators before raising
                for attached_motor_name in list(self.motors.keys())[:list(self.motors.keys()).index(motor_name)]:
                    try:
                        self.controller.detach_actuator(self.dcu_name, attached_motor_name)
                    except Exception as e:
                        logging.error(f"Failed to detach actuator {attached_motor_name} during connect rollback: {e}")
                raise RuntimeError(
                    f"Failed to attach actuator '{motor_name}' for DCU '{self.dcu_name}'. "
                    f"CtrlChannel: {motor[CTRL_CHANNEL_INDEX]}, CAN ID: {motor[CAN_ID_INDEX]}, "
                    f"Type: {motor[ACTURATOR_TYPE_INDEX]}"
                )
        self.is_connected = True

    def reconnect(self):
        if self.is_connected:
            # Attempt to gracefully disconnect first
            try:
                self.disconnect()
            except RobotDeviceNotConnectedError:
                # If it was already not connected, that's fine for a reconnect
                pass
            except Exception as e:
                logging.error(f"Error during disconnect in reconnect: {e}")
                # Even if disconnect fails, proceed to stop and attempt connection
                self.controller.stop()
                self.is_connected = False

        self.connect()

    def disconnect(self):
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                f"DynamixelMotorsBus({self.port}) is not connected. Try running `motors_bus.connect()` first."
            )
        self.controller.stop()
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
            # If no specific motor names are provided, default to all motor names configured for this bus.
            motor_names = self.motor_names() # Assuming motor_names() is a method returning a list of names.

        # Ensure calculations are performed using floating-point numbers for precision.
        # Raw motor values (values[i]) are often integers (e.g., from encoders).
        values = values.astype(np.float32)

        for i, name in enumerate(motor_names):
            # Retrieve calibration parameters specific to the current motor.
            calib_idx = self.calibration["motor_names"].index(name)
            calib_mode = self.calibration["calib_mode"][calib_idx] # Indicates DEGREE or LINEAR calibration.

            if CalibrationMode[calib_mode] == CalibrationMode.DEGREE:
                # Parameters for rotational joints (degree-based calibration).
                drive_mode = self.calibration["drive_mode"][calib_idx] # Boolean, true if motor direction needs inversion.
                homing_offset = self.calibration["homing_offset"][calib_idx] # Raw value offset for zeroing. This value might have been auto-corrected.
                # acturator_type is used to fetch the correct resolution for this specific motor model.
                _,_,acturator_type = self.motors[name] 
                resolution = self.acturator_type_resolution[acturator_type] # Motor-specific resolution (e.g., encoder counts per revolution or similar unit that defines a full 360-degree turn).

                # Apply drive_mode inversion to the current raw value if necessary.
                # This standardizes the motor's direction of rotation. For example, ensures positive values
                # always correspond to clockwise rotation, regardless of how the motor is physically mounted or wired.
                current_value = values[i] # Work with a copy of the value for this iteration.
                if drive_mode:
                    current_value *= -1 # Invert the raw value if motor's natural rotation is opposite to convention.

                # Calculate the calibrated value in degrees.
                # This `calib_val` is what the position *would* be if the current `homing_offset` (possibly auto-corrected) were applied,
                # and then scaled to the standard [-180, 180] degree range.
                # (resolution / RESOLUTION_HALF_DIVISOR) represents the number of raw motor units corresponding to HALF_TURN_DEGREE (180 degrees).
                calib_val = (current_value + homing_offset) / (resolution / RESOLUTION_HALF_DIVISOR) * HALF_TURN_DEGREE
                
                # Check if this calculated `calib_val` is outside the broad acceptable range (e.g., [-270, 270]).
                # This check is primarily for safety and to catch extreme errors that auto-correction might not have resolved
                # or if auto-correction is not perfect.
                in_range = (calib_val > LOWER_BOUND_DEGREE) and (calib_val < UPPER_BOUND_DEGREE)

                # If `calib_val` is out of the broad range, it implies the `homing_offset` might still be incorrect
                # or the raw value is too extreme. The auto-correction logic aims to fix common full-turn errors.
                # The bounds for `factor` are calculated to determine if a full-turn correction could bring the value
                # into the nominal [-HALF_TURN_DEGREE, HALF_TURN_DEGREE] range.
                #
                # The goal is to find an integer 'factor' such that if we adjust the raw value by 'factor * resolution',
                # effectively changing the homing_offset ( new_homing_offset = homing_offset + factor * resolution ),
                # the resulting calibrated value falls within the *nominal* [-HALF_TURN_DEGREE, HALF_TURN_DEGREE] range (e.g. [-180,180]).
                #
                # The formula for the recalibrated value would be:
                #   recal_val = (current_value + homing_offset + factor * resolution) / (resolution / RESOLUTION_HALF_DIVISOR) * HALF_TURN_DEGREE
                # We want: -HALF_TURN_DEGREE <= recal_val <= HALF_TURN_DEGREE
                # Solving this inequality for 'factor' yields the following `low_factor` and `upp_factor` bounds:
                low_factor = (-(resolution / RESOLUTION_HALF_DIVISOR) - current_value - homing_offset) / resolution
                upp_factor = ((resolution / RESOLUTION_HALF_DIVISOR) - current_value - homing_offset) / resolution

            elif CalibrationMode[calib_mode] == CalibrationMode.LINEAR:
                # Parameters for linear joints (percentage-based calibration).
                start_pos = self.calibration["start_pos"][calib_idx] # Raw value corresponding to 0%.
                end_pos = self.calibration["end_pos"][calib_idx]     # Raw value corresponding to 100%.
                # Note: `drive_mode` is not typically applied to linear joints in this calibration scheme.
                # `homing_offset` is also not directly used for linear in the same way as degree; `start_pos` and `end_pos` define the range.
                current_value = values[i] # Original raw value for this motor.
                # acturator_type is needed to fetch the correct resolution for this specific motor model,
                # used here for linear factor calculation if resolution is involved in linear adjustments.
                _,_,acturator_type = self.motors[name] 

                # Calculate the current position as a percentage using existing start/end positions.
                # This `calib_val` is used to check if an adjustment to calibration parameters is needed.
                # (end_pos - start_pos) must not be zero; if it is, the range is undefined.
                denominator = end_pos - start_pos
                if denominator == 0:
                    # This case implies a misconfiguration or a joint that isn't meant to move.
                    # Default to 0 or an error state; here, 0 is used, and range check might flag it.
                    calib_val = 0.0
                else:
                    calib_val = (current_value - start_pos) / denominator * PERCENTAGE_CONVERSION_FACTOR
                
                # Check if this `calib_val` is outside the broad acceptable percentage range (e.g., [-10, 110]).
                in_range = (calib_val > LOWER_BOUND_LINEAR) and (calib_val < UPPER_BOUND_LINEAR)

                # If `calib_val` is out of range, calculate a `factor` for correction.
                # For linear actuators, 'resolution' is used here to define a significant quantum for adjustment.
                # The goal is to find an integer 'factor' such that if we adjust `current_value` by `factor * resolution`
                # (which implies adjusting the effective zero point, as `homing_offset` is used for linear in this context),
                # the new `calib_val` falls within the nominal [0, PERCENTAGE_CONVERSION_FACTOR] range.
                #
                # The formula for the recalibrated value, if `current_value` were adjusted by `factor * resolution`, effectively means
                # `homing_offset` is adjusted by `factor * resolution`.
                # If calib_val = ( (current_value + homing_offset_adjustment) - start_pos) / (end_pos - start_pos) * PERCENTAGE_CONVERSION_FACTOR
                #   where homing_offset_adjustment = factor * resolution.
                # We want: 0 <= calib_val <= PERCENTAGE_CONVERSION_FACTOR
                # Solving for 'factor' (applied to homing_offset, which is added to current_value):
                #   Let effective_value = current_value + factor * resolution
                #   0 <= (effective_value - start_pos) / (end_pos - start_pos) * PERCENTAGE_CONVERSION_FACTOR <= PERCENTAGE_CONVERSION_FACTOR
                #   0 <= (effective_value - start_pos) <= (end_pos - start_pos)
                #   start_pos <= effective_value <= end_pos
                #   start_pos <= current_value + factor * resolution <= end_pos
                #   (start_pos - current_value) / resolution <= factor <= (end_pos - current_value) / resolution
                # This 'resolution' for linear actuators might represent the smallest discrete measurable unit or a similar quantum.
                resolution = self.acturator_type_resolution.get(acturator_type, 1.0) # Default to 1.0 if not found for this actuator type.
                if resolution == 0: resolution = 1.0 # Avoid division by zero if resolution is misconfigured to 0.
                
                low_factor = (start_pos - current_value) / resolution
                upp_factor = (end_pos - current_value) / resolution
            
            if not in_range:
                # If the initially calculated `calib_val` was out of the broad range, a correction is needed.
                # Find the smallest integer `factor` (number of full turns or equivalent linear shifts)
                # that should bring the value into the nominal range.
                if low_factor < upp_factor:
                    factor = math.ceil(low_factor)
                    # If `math.ceil(low_factor)` overshoots `upp_factor`, it means no single integer `factor`
                    # can bring the value into the desired range based on the derived inequalities.
                    # This could indicate extreme raw values or problematic calibration parameters.
                    if factor > upp_factor:
                        raise ValueError(f"No integer 'factor' found between calculated bounds [{low_factor=}, {upp_factor=}] for motor {name}. Cannot autocorrect.")
                else: # low_factor >= upp_factor. This can occur if value is very far off or bounds are narrow.
                    factor = math.ceil(upp_factor) # Try to approach from the other side.
                    if factor > low_factor: # Similar check for overshoot.
                        raise ValueError(f"No integer 'factor' found between calculated bounds [{low_factor=}, {upp_factor=}] for motor {name} (inverted bounds). Cannot autocorrect.")

                # Prepare logging messages.
                current_calib_val_str = f"{calib_val:.2f}" # Format for readability.
                if CalibrationMode[calib_mode] == CalibrationMode.DEGREE:
                    out_of_range_str = f"{LOWER_BOUND_DEGREE} < {current_calib_val_str} < {UPPER_BOUND_DEGREE} degrees"
                    # The target is the nominal range after correction.
                    in_range_str = f"nominal range [-{HALF_TURN_DEGREE}, {HALF_TURN_DEGREE}] degrees"
                elif CalibrationMode[calib_mode] == CalibrationMode.LINEAR:
                    out_of_range_str = f"{LOWER_BOUND_LINEAR} < {current_calib_val_str} < {UPPER_BOUND_LINEAR} %"
                    in_range_str = f"nominal range [0, {PERCENTAGE_CONVERSION_FACTOR}] %"
                
                logging.warning(
                    f"Auto-correct calibration of motor '{name}'. Shifting homing_offset by {factor} * {resolution:.2f} units. "
                    f"Original calculated value {current_calib_val_str} was out of expected range ({out_of_range_str}). Aiming for {in_range_str}."
                )

                # Apply the correction: Adjust the 'homing_offset' by 'factor * resolution'.
                # This modification of `self.calibration["homing_offset"]` is persistent for the current session
                # and will be used by all subsequent calls to `apply_calibration` and `revert_calibration`.
                # For linear mode, this effectively shifts the zero point if homing_offset is conceptually used,
                # or adjusts start_pos/end_pos if those were directly modified (though current code adjusts homing_offset).
                # It's assumed 'homing_offset' is the primary parameter to adjust for both modes in auto-correction.
                self.calibration["homing_offset"][calib_idx] += resolution * factor        

    def apply_calibration(self, values: np.ndarray | list, motor_names: list[str] | None):
        #print(f"Applying calibration to {motor_names} {values}")
        if not self.calibration:
            raise ValueError("Calibration data not set.")

        if motor_names is None:
            # If no specific motor names are provided, default to all motor names configured for this bus.
            motor_names = self.motor_names() # Assuming motor_names() is a method returning a list of names.
        # If motor_names was provided (even as an empty list), it's used directly.
        # Otherwise, self.motor_names() is used (as populated above).

        # Ensure calculations are performed using floating-point numbers for precision.
        # Raw motor values (values[i]) are often integers.
        values = values.astype(np.float32)

        for i, name in enumerate(motor_names):
            # Retrieve calibration parameters for the current motor.
            # These parameters are assumed to be correct (e.g., after potential auto-correction by `autocorrect_calibration`).
            calib_idx = self.calibration["motor_names"].index(name)
            calib_mode = self.calibration["calib_mode"][calib_idx] # Indicates DEGREE or LINEAR calibration.

            if CalibrationMode[calib_mode] == CalibrationMode.DEGREE:
                # Parameters for rotational joints (degree-based calibration).
                drive_mode = self.calibration["drive_mode"][calib_idx] # Boolean, true if motor direction needs inversion.
                homing_offset = self.calibration["homing_offset"][calib_idx] # The calibrated zero point for the motor's raw value.
                _,_,acturator_type = self.motors[name]
                resolution = self.acturator_type_resolution[acturator_type] # Motor-specific resolution (raw units for a full 360-degree turn).

                # Step 1: Apply drive mode inversion to the raw motor value if necessary.
                # This standardizes the motor's direction of rotation before applying other calibration logic.
                # E.g., if `drive_mode` is true, a raw value of 100 becomes -100.
                current_value = values[i] # Work with a copy for clarity in steps
                if drive_mode:
                    current_value *= -1

                # Step 2: Apply the homing offset to the (potentially inverted) raw value.
                # This shifts the motor's reported value so that its calibrated zero position aligns with the logical zero.
                # For example, if raw value is 500, and homing_offset is -450 (meaning 450 is the "true" zero),
                # current_value becomes 500 + (-450) = 50 (if not inverted by drive_mode).
                # The `homing_offset` itself would have been corrected by `autocorrect_calibration` if jumps were detected.
                current_value += homing_offset

                # Step 3: Scale the offset-adjusted value to degrees.
                # (resolution / RESOLUTION_HALF_DIVISOR) represents the number of raw motor units that correspond to HALF_TURN_DEGREE (180 degrees).
                # Dividing the adjusted value by this quantity gives the position as a fraction of a half-turn.
                # Multiplying by HALF_TURN_DEGREE converts this fraction into degrees.
                # Example: If adjusted value is `X`, resolution is 4096 (for 360 deg), RESOLUTION_HALF_DIVISOR is 2.0, HALF_TURN_DEGREE is 180.
                # Raw units for 180 deg = 4096 / 2.0 = 2048.
                # Position in degrees = (X / 2048) * 180.
                # This maps the motor's full rotation into a standard +/- degree range (e.g., often centered around 0).
                calibrated_value = current_value / (resolution / RESOLUTION_HALF_DIVISOR) * HALF_TURN_DEGREE
                values[i] = calibrated_value # Update the original array with the final calibrated value.

                # Step 4: Final check: ensure the now-calibrated degree value is within acceptable operational limits.
                # These bounds (LOWER_BOUND_DEGREE, UPPER_BOUND_DEGREE) are typically wider than the nominal
                # [-HALF_TURN_DEGREE, HALF_TURN_DEGREE] range (e.g., [-180, 180]) to allow for some mechanical tolerance
                # but are primarily to catch gross errors or unexpected behavior post-calibration.
                if (values[i] < LOWER_BOUND_DEGREE) or (values[i] > UPPER_BOUND_DEGREE):
                    raise JointOutOfRangeError(
                        f"Wrong motor position range detected for {name} after applying calibration. "
                        f"Expected to be in nominal range of [-{HALF_TURN_DEGREE}, {HALF_TURN_DEGREE}] degrees, "
                        f"with a maximum allowed range of [{LOWER_BOUND_DEGREE}, {UPPER_BOUND_DEGREE}] degrees. "
                        f"Got {values[i]:.2f} degrees. " # Formatted for readability
                        "This could indicate persistent hardware issues, extreme miscalibration, or problems with the calibration parameters themselves. "
                        "Consider recalibrating: `python lerobot/scripts/control_robot.py calibrate`"
                    )

            elif CalibrationMode[calib_mode] == CalibrationMode.LINEAR:
                # Parameters for linear joints (percentage-based calibration).
                start_pos = self.calibration["start_pos"][calib_idx] # Raw motor value corresponding to 0%.
                end_pos = self.calibration["end_pos"][calib_idx]     # Raw motor value corresponding to 100%.
                                                                    # Note: `drive_mode` is not typically used for linear actuators in this scheme.
                                                                    # `homing_offset` is also not directly used; `start_pos` effectively serves this role for linear.
                current_value = values[i] # Original raw value.

                # Step 1: Rescale the raw motor value to a percentage range [0, PERCENTAGE_CONVERSION_FACTOR].
                # This is a linear interpolation (or extrapolation if outside calibrated range).
                # `(current_value - start_pos)`: The current raw value relative to the calibrated 0% point.
                # `(end_pos - start_pos)`: The total span of raw values that corresponds to the full (100%) linear motion.
                # The ratio of these two quantities gives the fractional position along the calibrated range.
                # Multiplying by PERCENTAGE_CONVERSION_FACTOR scales this fraction to the desired percentage unit (e.g., 0-100).
                # Ensure (end_pos - start_pos) is not zero to prevent division by zero errors.
                denominator = end_pos - start_pos
                if denominator == 0:
                    # Handle cases where start_pos and end_pos are the same, which would lead to division by zero.
                    # This might mean the joint isn't meant to move or is miscalibrated.
                    # Setting to 0% or raising an error are options. Here, we set to 0 and let range check catch if it's invalid.
                    calibrated_value = 0.0 
                else:
                    calibrated_value = (current_value - start_pos) / denominator * PERCENTAGE_CONVERSION_FACTOR
                
                values[i] = calibrated_value # Update the original array.

                # Step 2: Final check: ensure the calibrated percentage is within acceptable operational limits.
                # Similar to degree mode, these bounds allow for slight variations but catch larger errors.
                if (values[i] < LOWER_BOUND_LINEAR) or (values[i] > UPPER_BOUND_LINEAR):
                    raise JointOutOfRangeError(
                        f"Wrong motor position range detected for {name} after applying calibration. "
                        f"Expected to be in nominal range of [0, {PERCENTAGE_CONVERSION_FACTOR}] %, "
                        f"with a maximum allowed range of [{LOWER_BOUND_LINEAR}, {UPPER_BOUND_LINEAR}] %. "
                        f"Got {values[i]:.2f} %. " # Formatted for readability
                        "This could indicate hardware issues, miscalibration of start/end points, or sensor problems. "
                        "Consider recalibrating: `python lerobot/scripts/control_robot.py calibrate`"
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
                # range [-resolution//RESOLUTION_HALF_DIVISOR, resolution//RESOLUTION_HALF_DIVISOR] (e.g. [-2048, 2048])
                values[i] = values[i] / HALF_TURN_DEGREE * (resolution / RESOLUTION_HALF_DIVISOR)

                # Reverse nominal range [-resolution//RESOLUTION_HALF_DIVISOR, resolution//RESOLUTION_HALF_DIVISOR] to
                # range [-2**31, 2**31]
                values[i] -= homing_offset

                # Reverse direction of rotation if necessary
                if drive_mode:
                    values[i] *= -1

            elif CalibrationMode[calib_mode] == CalibrationMode.LINEAR:
                start_pos = self.calibration["start_pos"][calib_idx]
                end_pos = self.calibration["end_pos"][calib_idx]

                # Reverse rescale from nominal range [0, PERCENTAGE_CONVERSION_FACTOR] % to original position range
                values[i] = values[i] / PERCENTAGE_CONVERSION_FACTOR * (end_pos - start_pos) + start_pos

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
            a = self.controller.get_position(name)
            #print(f"Reading {data_name} for {name} reader_value {a}")
            values.append(reader(name))
        values = np.array(values)
        print(f"loaded {data_name} for {motor_names} {values}")
        if data_name in CALIBRATION_REQUIRED and self.calibration is not None:
            values = self.apply_calibration(values, motor_names)    
        return np.array(values)

    def write(
        self, 
        data_name: str, 
        values: int | float | np.ndarray, 
        motor_names: str | list[str] | None = None,
        target_pos: int | float | np.ndarray | None = None,
        target_vel: int | float | np.ndarray | None = None,
        target_eff: int | float | np.ndarray | None = None,
    ):
        """
        Writes data to the specified motors. Allows setting primary data (position, velocity, or effort)
        while optionally specifying secondary target values for the other two parameters of the MIT command.

        Args:
            data_name: The type of data to write ("position", "velocity", "effort").
            values: The primary values to write for the specified data_name.
            motor_names: A specific motor name or list of motor names. Defaults to all motors on the bus.
            target_pos: Optional target positions. Used if data_name is velocity or effort.
            target_vel: Optional target velocities. Used if data_name is position or effort.
            target_eff: Optional target efforts. Used if data_name is position or velocity.
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to the motors bus.")
        motor_names = motor_names or self.motor_names()
        if isinstance(motor_names, str):
            motor_names = [motor_names]
        
        num_motors = len(motor_names)

        # Helper to prepare an array for optional parameters
        def prepare_optional_param(param_val, default_val):
            if param_val is None:
                return [default_val] * num_motors
            if not isinstance(param_val, (list, np.ndarray)):
                return [param_val] * num_motors
            if len(param_val) != num_motors:
                raise ValueError(f"Length of optional parameter array must match the number of motor names ({num_motors}).")
            return np.array(param_val)

        # Prepare primary values array
        if not isinstance(values, (list, np.ndarray)):
            primary_values = np.array([values] * num_motors)
        else:
            primary_values = np.array(values)
            if len(primary_values) != num_motors:
                raise ValueError(f"Length of values array must match the number of motor names ({num_motors}).")

        # Apply revert_calibration only to the primary values if applicable
        if data_name in CALIBRATION_REQUIRED and self.calibration is not None:
            primary_values = self.revert_calibration(primary_values, motor_names)
        
        # Prepare optional parameter arrays
        # Note: Optional parameters are NOT reverted by calibration as they might be used as direct passthrough
        # or have different units/scaling not covered by the primary data calibration.
        # If they need calibration, they should be passed as primary `data_name`.
        positions_cmd = prepare_optional_param(target_pos, MIT_CMD_DEFAULT_POS)
        velocities_cmd = prepare_optional_param(target_vel, MIT_CMD_DEFAULT_VEL)
        efforts_cmd = prepare_optional_param(target_eff, MIT_CMD_DEFAULT_EFF)

        logging.debug(f"Preparing to write to motors: {motor_names}")
        logging.debug(f"  Primary data_name: {data_name}")
        logging.debug(f"  Primary values (after potential revert_calibration): {primary_values}")
        logging.debug(f"  Target positions cmd: {positions_cmd}")
        logging.debug(f"  Target velocities cmd: {velocities_cmd}")
        logging.debug(f"  Target efforts cmd: {efforts_cmd}")

        for i, name in enumerate(motor_names):
            current_pos = positions_cmd[i]
            current_vel = velocities_cmd[i]
            current_eff = efforts_cmd[i]

            if data_name == DATA_NAME_POSITION:
                current_pos = primary_values[i]
            elif data_name == DATA_NAME_VELOCITY:
                current_vel = primary_values[i]
            elif data_name == DATA_NAME_EFFORT:
                current_eff = primary_values[i]
            else:
                raise ValueError(f"Unknown data_name: {data_name}")

            logging.debug(
                f"  Motor '{name}': cmd_pos={current_pos:.4f}, cmd_vel={current_vel:.4f}, cmd_eff={current_eff:.4f}, kp={self.kp}, kd={self.kd}"
            )
            ret = self.controller.set_mit_cmd(name, current_pos, current_vel, current_eff, self.kp, self.kd)
            if ret == 0: # Assuming 0 is success
                logging.info(f"Motor '{name}': Successfully set {data_name} (pos={current_pos:.2f}, vel={current_vel:.2f}, eff={current_eff:.2f}). Result code: {ret}")
            else: # Non-zero indicates a warning/error
                 logging.warning(f"Motor '{name}': set_mit_cmd returned non-zero status: {ret}. Pos={current_pos:.2f}, Vel={current_vel:.2f}, Eff={current_eff:.2f}")
