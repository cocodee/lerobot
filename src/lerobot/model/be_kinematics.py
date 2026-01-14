# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
import logging

logger = logging.getLogger(__name__)

class BeRobotKinematics:
    """Robot kinematics using placo library for forward and inverse kinematics.

    Enhanced with multi-priority task system to solve jump and humanization issues.

    Task Priority Structure:
    - Priority 0 (Hard Constraints): Joint Limits, Velocity Limits
    - Priority 1 (Primary Soft Task): PositionTask (end-effector position, weight=1.0)
    - Priority 2 (Secondary Soft Task): OrientationTask (end-effector orientation, weight=0.6-0.8)
    - Priority 3 (Regularization & Posture): PostureTask, JointDeltaLimit
    """

    def __init__(
        self,
        urdf_path: str,
        target_frame_name: str = "gripper_frame_link",
        joint_names: list[str] = None,
        position_weight: float = 1.0,
        orientation_weight: float = 0.7,
        posture_weight: float = 0.1,
        posture_reference: np.ndarray = None,
        velocity_limits: dict[str, float] = None,
        joint_delta_limit: float = 0.1,
    ):
        """
        Initialize placo-based kinematics solver with multi-priority tasks.

        Args:
            urdf_path: Path to the robot URDF file
            target_frame_name: Name of the end-effector frame in the URDF
            joint_names: List of joint names to use for the kinematics solver
            position_weight: Weight for position task (Priority 1)
            orientation_weight: Weight for orientation task (Priority 2), recommended 0.6-0.8
            posture_weight: Weight for posture task (Priority 3)
            posture_reference: Reference joint configuration for humanization (in degrees)
            velocity_limits: Dict of joint velocity limits (in rad/s)
            joint_delta_limit: Maximum joint change per step (in radians)
        """
        try:
            import placo
        except ImportError as e:
            raise ImportError(
                "placo is required for BeRobotKinematics. "
                "Please install the optional dependencies of `kinematics` in the package."
            ) from e

        self.robot = placo.RobotWrapper(urdf_path)
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.mask_fbase(True)  # Fix the base

        self.target_frame_name = target_frame_name

        # Set joint names
        self.joint_names = list(self.robot.joint_names()) if joint_names is None else joint_names

        # Parameters
        self.position_weight = position_weight
        self.orientation_weight = orientation_weight
        self.posture_weight = posture_weight
        self.velocity_limits = velocity_limits or {}
        self.joint_delta_limit = joint_delta_limit

        # Previous joint positions for delta limiting
        self.prev_joint_pos = None

        # Posture reference (convert to radians if provided)
        if posture_reference is not None:
            self.posture_reference = np.deg2rad(posture_reference)
        else:
            # Use middle of joint limits as default posture
            self.posture_reference = self._get_default_posture()

        # ========================================
        # Priority 0: Hard Constraints
        # ========================================

        # Enable joint limits (hard constraint)
        self.solver.enable_joint_limits(True)

        # Velocity limits (if provided)
        if self.velocity_limits:
            for joint_name, limit in self.velocity_limits.items():
                if joint_name in self.joint_names:
                    try:
                        self.solver.set_velocity_limit(joint_name, limit)
                        logger.info(f"[Priority 0] Set velocity limit for {joint_name}: {limit} rad/s")
                    except Exception as e:
                        logger.warning(f"Failed to set velocity limit for {joint_name}: {e}")

        # ========================================
        # Priority 1: Position Task (Primary Soft Task)
        # ========================================

        self.position_task = self.solver.add_position_task(
            self.target_frame_name,
            np.zeros(3),
            weight=self.position_weight
        )
        logger.info(f"[Priority 1] Position task created with weight={self.position_weight}")

        # ========================================
        # Priority 2: Orientation Task (Secondary Soft Task)
        # ========================================

        self.orientation_task = self.solver.add_orientation_task(
            self.target_frame_name,
            np.eye(3),
            weight=self.orientation_weight
        )
        logger.info(f"[Priority 2] Orientation task created with weight={self.orientation_weight}")

        # ========================================
        # Priority 3: Posture Task (Regularization)
        # ========================================

        self.posture_task = self.solver.add_posture_task(
            self.posture_reference,
            weight=self.posture_weight
        )
        logger.info(f"[Priority 3] Posture task created with weight={self.posture_weight}")

        # Mask unused DOFs
        for joint_name in self.robot.joint_names():
            if joint_name not in self.joint_names:
                self.solver.mask_dof(joint_name)

    def _get_default_posture(self):
        """Get default posture (middle of joint limits)."""
        posture = []
        for joint_name in self.joint_names:
            try:
                lower, upper = self.solver.get_joint_limits(joint_name)
                posture.append((lower + upper) / 2.0)
            except Exception:
                posture.append(0.0)  # Fallback to zero
        return np.array(posture)

    def forward_kinematics(self, joint_pos_deg):
        """
        Compute forward kinematics for given joint configuration given the target frame name in the constructor.

        Args:
            joint_pos_deg: Joint positions in degrees (numpy array)

        Returns:
            4x4 transformation matrix of the end-effector pose
        """

        # Convert degrees to radians
        joint_pos_rad = np.deg2rad(joint_pos_deg[: len(self.joint_names)])
        logger.info(f"joint_pos_rad: {joint_pos_rad}")
        # Update joint positions in placo robot
        for i, joint_name in enumerate(self.joint_names):
            self.robot.set_joint(joint_name, joint_pos_rad[i])

        # Update kinematics
        self.robot.update_kinematics()

        # Get the transformation matrix
        return self.robot.get_T_world_frame(self.target_frame_name)

    def inverse_kinematics(
        self,
        current_joint_pos,
        desired_ee_pose,
        position_weight: float = None,
        orientation_weight: float = None,
        use_posture: bool = True,
    ):
        """
        Compute inverse kinematics using placo solver with multi-priority tasks.

        Args:
            current_joint_pos: Current joint positions in degrees (used as initial guess)
            desired_ee_pose: Target end-effector pose as a 4x4 transformation matrix
            position_weight: Override position task weight (optional)
            orientation_weight: Override orientation task weight (optional)
            use_posture: Whether to enable posture task for humanization

        Returns:
            Joint positions in degrees that achieve the desired end-effector pose
        """

        # Convert current joint positions to radians for initial guess
        current_joint_rad = np.deg2rad(current_joint_pos[: len(self.joint_names)])
        logger.info(f"current_joint_rad: {current_joint_rad}")

        # Store previous joint position for delta limiting
        if self.prev_joint_pos is None:
            self.prev_joint_pos = current_joint_rad.copy()

        # Set current joint positions as initial guess
        for i, joint_name in enumerate(self.joint_names):
            self.robot.set_joint(joint_name, current_joint_rad[i])

        # Extract desired position and orientation
        desired_pos = desired_ee_pose[:3, 3]
        desired_rot = desired_ee_pose[:3, :3]

        # ========================================
        # Priority 1: Update Position Task
        # ========================================

        pos_w = position_weight if position_weight is not None else self.position_weight
        self.position_task.set_target(desired_pos)
        self.position_task.configure(pos_w, 0.0)  # Position only
        logger.info(f"[Priority 1] Position task set to {desired_pos}, weight={pos_w}")

        # ========================================
        # Priority 2: Update Orientation Task
        # ========================================

        ori_w = orientation_weight if orientation_weight is not None else self.orientation_weight
        self.orientation_task.set_target(desired_rot)
        self.orientation_task.configure(0.0, ori_w)  # Orientation only
        logger.info(f"[Priority 2] Orientation task set with weight={ori_w}")

        # ========================================
        # Priority 3: Posture Task (Optional)
        # ========================================

        if use_posture:
            self.posture_task.set_target(self.posture_reference)
            self.posture_task.configure(self.posture_weight)
            logger.info(f"[Priority 3] Posture task enabled")
        else:
            self.posture_task.configure(0.0)  # Disable
            logger.info(f"[Priority 3] Posture task disabled")

        # ========================================
        # Solve IK with multi-priority tasks
        # ========================================

        self.solver.solve(True)
        self.solver.dump_status()
        self.robot.update_kinematics()

        # ========================================
        # Apply Joint Delta Limit (Priority 3)
        # ========================================

        joint_pos_rad = []
        for joint_name in self.joint_names:
            joint = self.robot.get_joint(joint_name)
            # Limit delta from previous position
            prev_val = self.prev_joint_pos[self.joint_names.index(joint_name)]
            delta = joint - prev_val
            delta_clamped = np.clip(delta, -self.joint_delta_limit, self.joint_delta_limit)
            joint_pos_rad.append(prev_val + delta_clamped)

        # Update previous position for next iteration
        self.prev_joint_pos = np.array(joint_pos_rad)

        # ========================================
        # Error Analysis
        # ========================================

        # Get actual pose from solver
        actual_pose = self.robot.get_T_world_frame(self.target_frame_name)

        # Position error
        pos_error = np.linalg.norm(desired_ee_pose[:3, 3] - actual_pose[:3, 3])

        # Orientation error (using Z-axis alignment)
        desired_z = desired_ee_pose[:3, 2]
        actual_z = actual_pose[:3, 2]
        alignment = np.dot(desired_z, actual_z)

        logger.info(f"[IK Debug] Pos Error: {pos_error*1000:.2f} mm | Z-Align: {alignment:.4f}")

        # ========================================
        # Convert back to degrees
        # ========================================

        joint_pos_deg = np.rad2deg(joint_pos_rad)

        # Preserve gripper position if present in current_joint_pos
        if len(current_joint_pos) > len(self.joint_names):
            result = np.zeros_like(current_joint_pos)
            result[: len(self.joint_names)] = joint_pos_deg
            result[len(self.joint_names) :] = current_joint_pos[len(self.joint_names) :]
            return result
        else:
            return joint_pos_deg

    def set_posture_reference(self, posture_deg: np.ndarray):
        """Update the reference posture for humanization.

        Args:
            posture_deg: Reference joint configuration in degrees
        """
        self.posture_reference = np.deg2rad(posture_deg)
        logger.info(f"Posture reference updated to {posture_deg}")
