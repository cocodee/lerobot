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

class RobotKinematics:
    """Robot kinematics using placo library for forward and inverse kinematics."""

    def __init__(
        self,
        urdf_path: str,
        target_frame_name: str = "gripper_frame_link",
        joint_names: list[str] = None,
    ):
        """
        Initialize placo-based kinematics solver.

        Args:
            urdf_path: Path to the robot URDF file
            target_frame_name: Name of the end-effector frame in the URDF
            joint_names: List of joint names to use for the kinematics solver
        """
        try:
            import placo
        except ImportError as e:
            raise ImportError(
                "placo is required for RobotKinematics. "
                "Please install the optional dependencies of `kinematics` in the package."
            ) from e

        self.robot = placo.RobotWrapper(urdf_path)
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.mask_fbase(True)  # Fix the base

        self.target_frame_name = target_frame_name

        # Set joint names
        self.joint_names = list(self.robot.joint_names()) if joint_names is None else joint_names

        # Initialize frame task for IK
        self.tip_frame = self.solver.add_frame_task(self.target_frame_name, np.eye(4))

        for joint_name in self.robot.joint_names():
            if joint_name in self.joint_names:
                # 如果是左臂关节，允许求解器移动它
                self.solver.enable_joint(joint_name, True)
            else:
                # 如果是其他关节（右臂、腿、头等），禁止求解器移动它
                self.solver.enable_joint(joint_name, False)

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
        self, current_joint_pos, desired_ee_pose, position_weight=1.0, orientation_weight=0.01
    ):
        """
        Compute inverse kinematics using placo solver.

        Args:
            current_joint_pos: Current joint positions in degrees (used as initial guess)
            desired_ee_pose: Target end-effector pose as a 4x4 transformation matrix
            position_weight: Weight for position constraint in IK
            orientation_weight: Weight for orientation constraint in IK, set to 0.0 to only constrain position

        Returns:
            Joint positions in degrees that achieve the desired end-effector pose
        """

        # Convert current joint positions to radians for initial guess
        current_joint_rad = np.deg2rad(current_joint_pos[: len(self.joint_names)])
        logger.info(f"current_joint_rad: {current_joint_rad}")
        # Set current joint positions as initial guess
        for i, joint_name in enumerate(self.joint_names):
            self.robot.set_joint(joint_name, current_joint_rad[i])

        # Update the target pose for the frame task
        self.tip_frame.T_world_frame = desired_ee_pose

        # Configure the task based on position_only flag
        self.tip_frame.configure(self.target_frame_name, "soft", position_weight, orientation_weight)

        # Solve IK
        self.solver.solve(True)
        self.robot.update_kinematics()

        # === 新增：误差分析 ===
        # 1. 获取 IK 算出来的末端实际位置 (Actual Pose)
        actual_pose = self.robot.get_T_world_frame(self.target_frame_name)
        
        # 2. 计算位置误差 (Position Error) - 欧几里得距离
        # 提取 desired 和 actual 的位置向量 (前3行第4列)
        pos_error = np.linalg.norm(desired_ee_pose[:3, 3] - actual_pose[:3, 3])
        
        # 3. 计算旋转误差 (Orientation Error) - 简单的近似
        # 计算两个旋转矩阵的迹(Trace)的差异，或者计算轴角差
        # 这里用更直观的方式：打印 Z 轴朝向的偏差
        desired_z = desired_ee_pose[:3, 2]
        actual_z = actual_pose[:3, 2]
        # 计算点积，1.0 表示完全重合，0 表示垂直，-1 表示反向
        alignment = np.dot(desired_z, actual_z) 
        
        logger.info(f"[IK Debug] Pos Error: {pos_error*1000:.2f} mm | Z-Align: {alignment:.4f}")
        # Extract joint positions
        joint_pos_rad = []
        for joint_name in self.joint_names:
            joint = self.robot.get_joint(joint_name)
            joint_pos_rad.append(joint)

        # Convert back to degrees
        joint_pos_deg = np.rad2deg(joint_pos_rad)

        # Preserve gripper position if present in current_joint_pos
        if len(current_joint_pos) > len(self.joint_names):
            result = np.zeros_like(current_joint_pos)
            result[: len(self.joint_names)] = joint_pos_deg
            result[len(self.joint_names) :] = current_joint_pos[len(self.joint_names) :]
            return result
        else:
            return joint_pos_deg
