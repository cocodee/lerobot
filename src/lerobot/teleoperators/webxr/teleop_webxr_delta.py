"""
WebXR Delta Teleoperator - Wrapper that converts WebXR poses to delta actions.

This wraps WebxrTeleop and WebXRIntentTranslator to provide a consistent
interface with other teleoperators (outputting delta actions).
"""
import logging
import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import Any, Optional

from lerobot.teleoperators.webxr.teleop_webxr import WebxrTeleop
from lerobot.robots.sim_robot_panda.webxr_intent_translator import WebXRIntentTranslator
from lerobot.teleoperators.webxr.configuration_webxr import WebxrTeleopConfig
from lerobot.teleoperators import Teleoperator


logger = logging.getLogger(__name__)


class WebxrDeltaTeleop(Teleoperator):
    """
    WebXR teleoperator that outputs delta actions like other teleop devices.

    This class wraps WebxrTeleop and WebXRIntentTranslator to convert WebXR
    absolute poses into delta actions (position + rotation deltas).

    Key difference from WebxrTeleop:
    - WebxrTeleop: outputs {x, y, z, qx, qy, qz, qw, mode, type} (absolute pose)
    - WebxrDeltaTeleop: outputs {delta_x, delta_y, delta_z, delta_qx, ...} (delta)

    Usage:
        teleop = WebxrDeltaTeleop(config)
        teleop.connect()
        teleop.set_robot(robot)  # Optional: enables automatic pose updates
        action = teleop.get_action()  # Returns delta action
    """

    config_class = WebxrTeleopConfig
    name = "webxr_delta"
    robot_type = "sim_robot"

    def __init__(self, config: WebxrTeleopConfig):
        super().__init__(config)
        self.config = config

        # Create the underlying WebXR teleoperator
        self.webxr_teleop = WebxrTeleop(config)

        # Create the intent translator for pose conversion
        # TODO: Load xr_to_robot_matrix from config if needed
        self.translator = WebXRIntentTranslator()

        # Current robot end-effector pose (4x4 homogeneous matrix)
        self.current_ee_pose: Optional[np.ndarray] = None

        # Previous target pose for delta calculation
        self.prev_target_pose: Optional[np.ndarray] = None

        # Mode tracking
        self.current_mode = "IDLE"
        self.prev_mode = "IDLE"

        # Robot reference (optional, for auto-updating current pose)
        self._robot = None

        if self.config.robot_type is not None and self.config.robot_type != "":
            self.robot_type = self.config.robot_type

    @property
    def action_features(self) -> dict:
        """
        Define the output action space (delta format).

        Returns:
            Dictionary describing the delta action space.
        """
        if self.config.use_gripper:
            return {
                "dtype": "float32",
                "shape": (8,),
                "names": {
                    "delta_x": 0, "delta_y": 1, "delta_z": 2,
                    "delta_qx": 3, "delta_qy": 4, "delta_qz": 5, "delta_qw": 6,
                    "gripper": 7
                },
            }
        else:
            return {
                "dtype": "float32",
                "shape": (7,),
                "names": {
                    "delta_x": 0, "delta_y": 1, "delta_z": 2,
                    "delta_qx": 3, "delta_qy": 4, "delta_qz": 5, "delta_qw": 6
                }
            }

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        """Send feedback to the teleoperator (not used for WebXR)."""
        pass

    @property
    def feedback_features(self) -> dict:
        """Return feedback features (empty for WebXR)."""
        return {}

    @property
    def is_connected(self) -> bool:
        """Check if the teleoperator is connected."""
        return self.webxr_teleop.is_connected

    @property
    def is_calibrated(self) -> bool:
        """Check if the teleoperator is calibrated."""
        return True

    def connect(self) -> None:
        """Connect to the WebXR teleoperator."""
        self.webxr_teleop.connect()
        logger.info("WebxrDeltaTeleop connected")

    def disconnect(self) -> None:
        """Disconnect from the WebXR teleoperator."""
        self.webxr_teleop.disconnect()
        logger.info("WebxrDeltaTeleop disconnected")

    def calibrate(self) -> None:
        """Calibrate the teleoperator (no-op for WebXR)."""
        pass

    def configure(self):
        """Configure the teleoperator."""
        pass

    def set_robot(self, robot) -> None:
        """
        Set the robot reference for automatic pose updates.

        If set, get_action() will automatically fetch the current EE pose
        from the robot instead of requiring manual update_current_pose() calls.

        Args:
            robot: Robot instance with current_ee_pos attribute or kinematics
        """
        self._robot = robot
        logger.info("WebxrDeltaTeleop: robot reference set for automatic pose updates")

    def update_current_pose(self, current_ee_pose: np.ndarray) -> None:
        """
        Update the current robot end-effector pose.

        This can be called manually if robot reference is not set.
        Otherwise, get_action() will automatically fetch from robot.

        Args:
            current_ee_pose: 4x4 homogeneous transformation matrix
        """
        self.current_ee_pose = current_ee_pose.copy()

    def _get_current_ee_pose(self) -> Optional[np.ndarray]:
        """
        Get current EE pose from robot or cached value.

        Returns:
            4x4 homogeneous transformation matrix or None
        """
        # Try to get from robot if reference is set
        if self._robot is not None:
            # Check if robot has current_ee_pos (computed during send_action)
            if hasattr(self._robot, 'current_ee_pos') and self._robot.current_ee_pos is not None:
                return self._robot.current_ee_pos

            # Otherwise, try to compute from joint positions using kinematics
            if hasattr(self._robot, 'kinematics'):
                try:
                    # Try different methods to get joint positions
                    if hasattr(self._robot, 'get_present_joint_state'):
                        # SimRobotPandaHil style: returns {name: degrees}
                        joint_state = self._robot.get_present_joint_state()
                        # Get joint names from robot or use keys from joint_state
                        if hasattr(self._robot, 'get_joint_names'):
                            from lerobot.utils.robot_utils import get_joint_names
                            joint_names = get_joint_names(self._robot)
                        else:
                            joint_names = list(joint_state.keys())
                        joint_pos = np.array([joint_state[name] for name in joint_names])
                        return self._robot.kinematics.forward_kinematics(joint_pos)
                    elif hasattr(self._robot, 'observation') and 'agent_pos' in self._robot.observation:
                        # Gym style: observation contains agent_pos
                        joint_pos = self._robot.observation['agent_pos']
                        return self._robot.kinematics.forward_kinematics(joint_pos)
                except Exception as e:
                    logger.warning(f"Failed to compute current EE pose from robot: {e}")

        # Fall back to manually set pose
        return self.current_ee_pose

    def get_action(self) -> dict[str, Any]:
        """
        Get delta action from WebXR input.

        Automatically fetches current EE pose from robot if robot reference is set.

        Returns:
            Dictionary containing delta action:
            - delta_x, delta_y, delta_z: Position deltas
            - delta_qx, delta_qy, delta_qz, delta_qw: Rotation delta (quaternion)
            - gripper: Gripper action (0-2)
            - mode: Current WebXR mode (for debugging)
        """
        # Auto-fetch current EE pose if robot reference is set
        if self._robot is not None:
            self.current_ee_pose = self._get_current_ee_pose()

        # Get raw WebXR pose
        webxr_action = self.webxr_teleop.get_action()

        # Extract mode
        self.current_mode = webxr_action.get("mode", webxr_action.get("m", "IDLE"))
        gripper_val = webxr_action.get("gripper", webxr_action.get("g", 1.0))

        # Handle IDLE mode - return zero deltas
        if self.current_mode == "IDLE" or self.current_ee_pose is None:
            return self._zero_action(gripper_val)

        # Check for mode transition - reset prev_target_pose
        if self.current_mode != self.prev_mode:
            self.prev_target_pose = self.current_ee_pose.copy()
            logger.info(f"Mode changed: {self.prev_mode} -> {self.current_mode}, reset prev_target_pose")

        self.prev_mode = self.current_mode

        # Normalize frame for translator (handle both old and new formats)
        frame = self._normalize_frame(webxr_action)

        # Use WebXRIntentTranslator to compute target pose
        target_pose = self.translator.update(frame, self.current_ee_pose)

        # If translator returns None (shouldn't happen with non-IDLE mode), use current pose
        if target_pose is None:
            target_pose = self.current_ee_pose.copy()

        # Initialize prev_target_pose on first action
        if self.prev_target_pose is None:
            self.prev_target_pose = self.current_ee_pose.copy()

        # Calculate delta from prev_target to current target
        # This is the actual command to send to the robot
        delta_pos = target_pose[:3, 3] - self.prev_target_pose[:3, 3]

        # Calculate rotation delta
        current_rot = R.from_matrix(target_pose[:3, :3])
        prev_rot = R.from_matrix(self.prev_target_pose[:3, :3])
        delta_rot = prev_rot.inv() * current_rot
        delta_quat = delta_rot.as_quat()  # [x, y, z, w]

        # Update prev_target_pose for next iteration
        self.prev_target_pose = target_pose.copy()

        # Return delta action
        return {
            "delta_x": float(delta_pos[0]),
            "delta_y": float(delta_pos[1]),
            "delta_z": float(delta_pos[2]),
            "delta_qx": float(delta_quat[0]),
            "delta_qy": float(delta_quat[1]),
            "delta_qz": float(delta_quat[2]),
            "delta_qw": float(delta_quat[3]),
            "gripper": float(gripper_val),
            "mode": self.current_mode,
            "type": "webxr_delta"
        }

    def _normalize_frame(self, action: dict) -> dict:
        """
        Normalize action frame format for translator.

        Handles both old format (p, q, m) and new format (x, y, z, qx, qy, qz, qw, mode).
        """
        if "x" in action and "y" in action and "z" in action:
            # New format - convert to old format for translator
            return {
                "p": np.array([action["x"], action["y"], action["z"]]),
                "q": np.array([action["qx"], action["qy"], action["qz"], action["qw"]]),
                "mode": action.get("mode", "IDLE"),
                "type": action.get("type", "webxr")
            }
        else:
            # Old format - already compatible
            return {
                "p": action["p"],
                "q": action["q"],
                "mode": action.get("m", "IDLE"),
                "type": action.get("type", "webxr")
            }

    def _zero_action(self, gripper_val: float = 1.0) -> dict:
        """Return zero delta action."""
        return {
            "delta_x": 0.0,
            "delta_y": 0.0,
            "delta_z": 0.0,
            "delta_qx": 0.0,
            "delta_qy": 0.0,
            "delta_qz": 0.0,
            "delta_qw": 1.0,  # Identity quaternion
            "gripper": float(gripper_val),
            "mode": "IDLE",
            "type": "webxr_delta"
        }

    def reset(self) -> None:
        """Reset the teleoperator state."""
        self.prev_target_pose = None
        self.current_ee_pose = None
        self.prev_mode = "IDLE"
        self.current_mode = "IDLE"
        self.webxr_teleop.reset()
        logger.info(f"{self.name} teleoperator state reset")
