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

class SuperRobotKinematics:
    """Robot kinematics using placo library for forward and inverse kinematics.

    Enhanced version with:
    - Joint limits support
    - Singularity detection and handling
    - Adaptive IK solver with retries
    - Better error analysis and logging
    """

    # 默认关节限制 (Franka Panda/FR3 示例，单位：弧度)
    DEFAULT_JOINT_LIMITS = {
        "joint1": (-2.89, 2.89),
        "joint2": (-1.76, 1.76),
        "joint3": (-2.89, 2.89),
        "joint4": (-3.07, -0.06),
        "joint5": (-2.89, 2.89),
        "joint6": (-0.01, 3.75),
        "joint7": (-2.89, 2.89),
        # 兼容不同命名
        "panda_joint1": (-2.89, 2.89),
        "panda_joint2": (-1.76, 1.76),
        "panda_joint3": (-2.89, 2.89),
        "panda_joint4": (-3.07, -0.06),
        "panda_joint5": (-2.89, 2.89),
        "panda_joint6": (-0.01, 3.75),
        "panda_joint7": (-2.89, 2.89),
        "left_arm_joint_1": (-3.14, 3.14),
        "left_arm_joint_2": (-3.14, 3.14),
        "left_arm_joint_3": (-3.14, 3.14),
        "left_arm_joint_4": (-3.14, 3.14),
        "left_arm_joint_5": (-3.14, 3.14),
        "left_arm_joint_6": (-3.14, 3.14),
        "left_arm_joint_7": (-3.14, 3.14),
    }

    def __init__(
        self,
        urdf_path: str,
        target_frame_name: str = "gripper_frame_link",
        joint_names: list[str] = None,
        joint_limits: dict[str, tuple[float, float]] = None,
    ):
        """
        Initialize placo-based kinematics solver.

        Args:
            urdf_path: Path to the robot URDF file
            target_frame_name: Name of the end-effector frame in the URDF
            joint_names: List of joint names to use for the kinematics solver
            joint_limits: Optional dict of joint limits {joint_name: (lower, upper)} in radians
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

        # 优化求解器参数
        self.solver.max_iterations = 100  # 增加最大迭代次数
        self.solver.tol = 1e-4  # 设置容差

        self.target_frame_name = target_frame_name

        # Set joint names
        self.joint_names = list(self.robot.joint_names()) if joint_names is None else joint_names

        # Initialize frame task for IK
        self.tip_frame = self.solver.add_frame_task(self.target_frame_name, np.eye(4))

        for joint_name in self.robot.joint_names():
            if joint_name not in self.joint_names:
                # 如果是左臂关节，允许求解器移动它
                self.solver.mask_dof(joint_name)

        # 设置关节限制
        limits = joint_limits if joint_limits is not None else self.DEFAULT_JOINT_LIMITS
        for joint_name in self.joint_names:
            if joint_name in limits:
                lower, upper = limits[joint_name]
                try:
                    self.solver.set_joint_limits(joint_name, lower, upper)
                    logger.info(f"Set joint limits for {joint_name}: [{lower:.2f}, {upper:.2f}] rad")
                except Exception as e:
                    logger.warning(f"Failed to set joint limits for {joint_name}: {e}")

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

    def check_singularity(self, threshold=50.0):
        """
        检查当前关节配置是否接近奇异点。

        Args:
            threshold: 雅可比矩阵条件数的阈值，超过此值视为接近奇异点

        Returns:
            (is_singular, condition_number): 是否接近奇异点及其条件数
        """
        try:
            # 计算雅可比矩阵 (6 x n_joints)
            J = self.robot.jacobian(self.target_frame_name)

            # 只取位置部分 (前3行)
            J_pos = J[:3, :]

            # 计算条件数
            if J_pos.size > 0:
                condition_number = np.linalg.cond(J_pos)
                is_singular = condition_number > threshold
                return is_singular, condition_number
            return False, 0.0
        except Exception as e:
            logger.warning(f"Failed to check singularity: {e}")
            return False, 0.0

    def inverse_kinematics(
        self,
        current_joint_pos,
        desired_ee_pose,
        position_weight=1.0,
        orientation_weight=0.1,
        max_retries=3,
        singularity_threshold=50.0,
        adaptive_weights=True,
    ):
        """
        Compute inverse kinematics using placo solver with optimizations.

        Args:
            current_joint_pos: Current joint positions in degrees (used as initial guess)
            desired_ee_pose: Target end-effector pose as a 4x4 transformation matrix
            position_weight: Weight for position constraint in IK
            orientation_weight: Weight for orientation constraint in IK, set to 0.0 to only constrain position
            max_retries: Maximum number of IK solving retries on failure
            singularity_threshold: Threshold for detecting singular configurations
            adaptive_weights: Whether to adaptively adjust weights based on errors

        Returns:
            Joint positions in degrees that achieve the desired end-effector pose
        """

        # Convert current joint positions to radians for initial guess
        current_joint_rad = np.deg2rad(current_joint_pos[: len(self.joint_names)])

        # 设置初始关节位置
        for i, joint_name in enumerate(self.joint_names):
            self.robot.set_joint(joint_name, current_joint_rad[i])

        # 检查初始奇异点
        is_singular, cond_num = self.check_singularity(singularity_threshold)
        if is_singular:
            logger.warning(f"[IK] Starting near singularity! Condition number: {cond_num:.2f}")

        # 尝试多次求解
        best_solution = None
        best_error = float('inf')

        for attempt in range(max_retries):
            # Update the target pose for the frame task
            self.tip_frame.T_world_frame = desired_ee_pose

            # 自适应权重：根据误差调整
            curr_pos_weight = position_weight
            curr_ori_weight = orientation_weight

            if attempt > 0 and adaptive_weights:
                # 后续尝试降低位置权重，增加姿态权重
                curr_pos_weight = position_weight * 0.5
                curr_ori_weight = orientation_weight * 2.0
                logger.info(f"[IK] Retry {attempt + 1}: Adjusted weights - pos={curr_pos_weight:.3f}, ori={curr_ori_weight:.3f}")

            # Configure the task
            self.tip_frame.configure(self.target_frame_name, "soft", curr_pos_weight, curr_ori_weight)

            # Solve IK
            success = self.solver.solve(True)

            if not success:
                logger.warning(f"[IK] Attempt {attempt + 1}/{max_retries} failed")
                continue

            self.robot.update_kinematics()

            # === 误差分析 ===
            actual_pose = self.robot.get_T_world_frame(self.target_frame_name)
            pos_error = np.linalg.norm(desired_ee_pose[:3, 3] - actual_pose[:3, 3])

            # 旋转误差：使用旋转矩阵的 Frobenius 范数
            R_desired = desired_ee_pose[:3, :3]
            R_actual = actual_pose[:3, :3]
            rot_error = np.linalg.norm(R_desired - R_actual, 'fro')

            total_error = pos_error + 0.1 * rot_error  # 综合误差

            logger.info(
                f"[IK] Attempt {attempt + 1}/{max_retries}: "
                f"Pos Error: {pos_error * 1000:.2f} mm, "
                f"Rot Error: {rot_error:.4f}, "
                f"Cond: {cond_num:.2f}"
            )

            # 保留最佳解
            if total_error < best_error:
                best_error = total_error
                best_solution = []
                for joint_name in self.joint_names:
                    joint = self.robot.get_joint(joint_name)
                    best_solution.append(joint)

                # 如果误差足够小，提前退出
                if pos_error < 1e-3 and rot_error < 0.1:
                    logger.info(f"[IK] Converged at attempt {attempt + 1}")
                    break

            # 最后一次尝试失败，使用较小的步长
            if attempt < max_retries - 1:
                # 稍微调整初始猜测，避免局部最优
                for i, joint_name in enumerate(self.joint_names):
                    current_val = self.robot.get_joint(joint_name)
                    noise = np.random.normal(0, 0.01)  # 小幅随机扰动
                    self.robot.set_joint(joint_name, current_val + noise)

        # 检查是否找到有效解
        if best_solution is None:
            logger.error("[IK] All attempts failed! Returning current joint positions")
            return current_joint_pos

        # Convert back to degrees
        joint_pos_deg = np.rad2deg(best_solution)

        # Preserve gripper position if present in current_joint_pos
        if len(current_joint_pos) > len(self.joint_names):
            result = np.zeros_like(current_joint_pos)
            result[: len(self.joint_names)] = joint_pos_deg
            result[len(self.joint_names) :] = current_joint_pos[len(self.joint_names) :]
            return result
        else:
            return joint_pos_deg
