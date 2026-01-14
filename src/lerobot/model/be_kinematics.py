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
        try:
            import placo
        except ImportError as e:
            raise ImportError(
                "placo is required for BeRobotKinematics. "
                "Please install it via pip or from source."
            ) from e

        self.robot = placo.RobotWrapper(urdf_path)
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.mask_fbase(True)

        self.target_frame_name = target_frame_name
        self.joint_names = list(self.robot.joint_names()) if joint_names is None else joint_names

        self.position_weight = position_weight
        self.orientation_weight = orientation_weight
        self.posture_weight = posture_weight
        self.velocity_limits = velocity_limits or {}
        self.joint_delta_limit = joint_delta_limit
        self.prev_joint_pos = None

        if posture_reference is not None:
            self.posture_reference = np.deg2rad(posture_reference).astype(np.float64)
        else:
            self.posture_reference = self._get_default_posture().astype(np.float64)

        # ========================================
        # Priority 0: Hard Constraints
        # ========================================
        self.solver.enable_joint_limits(True)
        if self.velocity_limits:
            for joint_name, limit in self.velocity_limits.items():
                if joint_name in self.joint_names:
                    try:
                        self.solver.set_velocity_limit(joint_name, float(limit))
                    except Exception as e:
                        logger.warning(f"Failed to set velocity limit: {e}")

        # ========================================
        # Priority 1: Position Task
        # 修改点：去掉 weight= 参数，改用 configure
        # ========================================
        # 确保传入的是 float64 的 3D 向量
        initial_pos = np.zeros(3, dtype=np.float64)
        self.position_task = self.solver.add_position_task(
            self.target_frame_name, 
            initial_pos
        )
        # placo 的 Task 通常使用 configure 设置权重和类型 ("soft" 或 "hard")
        self.position_task.configure("position", "soft", self.position_weight)
        
        # ========================================
        # Priority 2: Orientation Task
        # 修改点：去掉 weight= 参数
        # ========================================
        initial_rot = np.eye(3, dtype=np.float64)
        self.orientation_task = self.solver.add_orientation_task(
            self.target_frame_name, 
            initial_rot
        )
        self.orientation_task.configure("orientation", "soft", self.orientation_weight)

        # ========================================
        # Priority 3: Posture Task
        # 修改点：add_posture_task 通常不需要参数，或者直接传引用
        # ========================================
        self.posture_task = self.solver.add_joints_task()
        
        # 为每个关节设置参考位置
        for i, joint_name in enumerate(self.joint_names):
            self.posture_task.set_joint(joint_name, float(self.posture_reference[i]))
            
        # 设置权重
        self.posture_task.configure("posture", "soft", float(self.posture_weight))
        logger.info(f"[Priority 3] Joints task created with weight={self.posture_weight}")
        
        # Mask unused DOFs
        for joint_name in self.robot.joint_names():
            if joint_name not in self.joint_names:
                self.solver.mask_dof(joint_name)

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
    def inverse_kinematics(
        self,
        current_joint_pos,
        desired_ee_pose,
        position_weight: float = None,
        orientation_weight: float = None,
        use_posture: bool = True,
    ):
        # 确保输入是 float64
        current_joint_rad = np.deg2rad(current_joint_pos[: len(self.joint_names)]).astype(np.float64)

        if self.prev_joint_pos is None:
            self.prev_joint_pos = current_joint_rad.copy()

        for i, joint_name in enumerate(self.joint_names):
            self.robot.set_joint(joint_name, current_joint_rad[i])

        # 确保位姿矩阵是 float64
        desired_pos = desired_ee_pose[:3, 3].astype(np.float64)
        desired_rot = desired_ee_pose[:3, :3].astype(np.float64)

        # 更新任务目标
        pos_w = float(position_weight if position_weight is not None else self.position_weight)
        self.position_task.target_world = desired_pos  # 改为属性赋值
        self.position_task.weight = pos_w

        ori_w = float(orientation_weight if orientation_weight is not None else self.orientation_weight)
        self.orientation_task.R_world_frame = desired_rot  # 改为属性赋值
        self.orientation_task.weight = ori_w

        if use_posture:
            # 更新每个关节的参考值（如果 posture_reference 发生变化）
            for i, joint_name in enumerate(self.joint_names):
                self.posture_task.set_joint(joint_name, float(self.posture_reference[i]))
            
            self.posture_task.weight = float(self.posture_weight)
            logger.info(f"[Priority 3] Posture task enabled")
        else:
            self.posture_task.weight = 0.0
            logger.info(f"[Priority 3] Posture task disabled")

        # 解算
        self.solver.solve(True)
        self.robot.update_kinematics()

        # ... (后续 delta limit 和返回逻辑保持不变，但确保运算使用 numpy float64)
        joint_pos_rad = []
        for joint_name in self.joint_names:
            joint_val = self.robot.get_joint(joint_name)
            idx = self.joint_names.index(joint_name)
            prev_val = self.prev_joint_pos[idx]
            delta = joint_val - prev_val
            delta_clamped = np.clip(delta, -self.joint_delta_limit, self.joint_delta_limit)
            joint_pos_rad.append(prev_val + delta_clamped)

        self.prev_joint_pos = np.array(joint_pos_rad)
        return np.rad2deg(joint_pos_rad) # 这里根据需要处理 gripper

    def _get_default_posture(self):
        posture = []
        for joint_name in self.joint_names:
            try:
                # 确保获取的是 float
                lower, upper = self.solver.get_joint_limits(joint_name)
                posture.append((lower + upper) / 2.0)
            except:
                posture.append(0.0)
        return np.array(posture, dtype=np.float64)