# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
# ... (License header omitted) ...

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
        orientation_weight: float = 0.1,
        posture_weight: float = 1e-3, # 默认姿态权重
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
        
        # 标志位：控制是否启用不同类型的任务
        self.use_position = True
        self.use_rotation = True
        self.use_posture = True # 始终开启初始化，通过权重控制是否生效

        self.robot = placo.RobotWrapper(urdf_path)
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.mask_fbase(True)

        self.target_frame_name = target_frame_name
        self.joint_names = list(self.robot.joint_names()) if joint_names is None else joint_names
        
        logger.info(f"joint_names: {self.joint_names}")
        
        self.position_weight = position_weight
        self.orientation_weight = orientation_weight
        self.posture_weight = posture_weight # 默认值，IK时可覆盖
        self.velocity_limits = velocity_limits or {}
        self.joint_delta_limit = joint_delta_limit
        self.prev_joint_pos = None

        # 初始化默认姿态 (用于 Fallback)
        if posture_reference is not None:
            self.posture_reference = np.deg2rad(posture_reference)
        else:
            self.posture_reference = self._get_default_posture()

        # ========================================
        # Priority 0: Hard Constraints (Velocity Limits)
        # ========================================
        if self.velocity_limits:
            for joint_name, limit in self.velocity_limits.items():
                if joint_name in self.joint_names:
                    try:
                        self.solver.set_velocity_limit(joint_name, float(limit))
                    except Exception as e:
                        logger.warning(f"Failed to set velocity limit: {e}")

        # ========================================
        # Priority 1: Position Task
        # ========================================
        if self.use_position:
            initial_pos = np.zeros(3)
            self.position_task = self.solver.add_position_task(
                self.target_frame_name, 
                initial_pos
            )
            self.position_task.configure("position", "soft", self.position_weight)
            logger.info(f"[Init] Position task enabled for {self.target_frame_name}")

        # ========================================
        # Priority 2: Orientation Task
        # ========================================
        if self.use_rotation:
            initial_rot = np.eye(3)
            self.orientation_task = self.solver.add_orientation_task(
                self.target_frame_name, 
                initial_rot
            )
            self.orientation_task.configure("orientation", "soft", self.orientation_weight)

        # ========================================
        # Priority 3: Posture / Joints Task (已修改)
        # ========================================
        # 这里只初始化任务，不设置具体目标，具体目标在 IK loop 中设置
        self.posture_task = self.solver.add_joints_task()
        self.posture_task.configure("posture", "soft", 0.0) # 初始权重为0，避免干扰

        # Mask unused DOFs
        for joint_name in self.robot.joint_names():
            if joint_name not in self.joint_names:
                self.solver.mask_dof(joint_name)

    def forward_kinematics(self, joint_pos_deg):
        """Standard Forward Kinematics"""
        joint_pos_rad = np.deg2rad(joint_pos_deg[: len(self.joint_names)])
        for i, joint_name in enumerate(self.joint_names):
            self.robot.set_joint(joint_name, joint_pos_rad[i])
        self.robot.update_kinematics()
        return self.robot.get_T_world_frame(self.target_frame_name)            

    def inverse_kinematics(
        self,
        current_joint_pos,
        desired_ee_pose,
        # === [修改] 新增参数 ===
        joint_task_weight: float = None, # 如果为 None，则使用 self.posture_weight
        target_joints: dict = None,      # {joint_name: angle_deg}
                position_weight: float = None,
        orientation_weight: float = None,
    ):
        # 1. 初始状态设置 (Initial Guess)
        current_joint_rad = np.deg2rad(current_joint_pos[: len(self.joint_names)])
        if self.prev_joint_pos is None:
            self.prev_joint_pos = current_joint_rad.copy()

        for i, joint_name in enumerate(self.joint_names):
            self.robot.set_joint(joint_name, current_joint_rad[i])
        self.robot.update_kinematics()

        # 2. 提取目标位姿
        desired_pos = desired_ee_pose[:3, 3]
        desired_rot = desired_ee_pose[:3, :3]

        # 3. 更新 Position Task
        if self.use_position:
            pos_w = float(position_weight if position_weight is not None else self.position_weight)
            self.position_task.target_world = desired_pos 
            self.position_task.configure("position", "soft", pos_w) # 使用 configure 动态设置权重

        # 4. 更新 Orientation Task
        if self.use_rotation:
            ori_w = float(orientation_weight if orientation_weight is not None else self.orientation_weight)
            self.orientation_task.R_world_frame = desired_rot
            self.orientation_task.configure("orientation", "soft", ori_w)

        # 5. 更新 Posture/Joints Task (核心修改)
        # 确定权重
        posture_w = float(joint_task_weight if joint_task_weight is not None else self.posture_weight)
        
        self.posture_task.configure("posture", "soft", posture_w)

        if posture_w > 1e-6:
            if target_joints is not None and len(target_joints) > 0:
                # 情况 A: 用户指定了特定的关节目标 (用于 Elbow Control 或部分关节控制)
                target_joints_rad = {k: np.deg2rad(v) for k, v in target_joints.items()}
                self.posture_task.set_joints(target_joints_rad)
            else:
                # 情况 B: 没有指定目标，但有权重 -> 使用默认/参考姿态 (用于 Regularization 防止漂移)
                fallback_joints = {}
                for i, name in enumerate(self.joint_names):
                    if i < len(self.posture_reference):
                        fallback_joints[name] = self.posture_reference[i]
                self.posture_task.set_joints(fallback_joints)
        else:
            # 权重为 0，清除约束
            self.posture_task.set_joints({})

        # 6. 求解
        # self.solver.regularization = 1e-5 # 可选：增加数值稳定性
        self.solver.solve(True)
        # self.solver.dump_status() # Debug用
        self.robot.update_kinematics()

        # 7. 提取结果 & 关节限位处理 (Delta Limit)
        joint_pos_rad = []
        
        # 简单模式：直接读取结果
        for joint_name in self.joint_names:
            joint = self.robot.get_joint(joint_name)
            joint_pos_rad.append(joint)

        # 8. (可选) 实现 Delta Limiting 防止跳变
        # 如果需要严格限制每帧变化量，可以在这里把 joint_pos_rad 和 self.prev_joint_pos 做 clip
        # 这里保留了你原有代码中 "else" 的逻辑，即直接返回解算结果
        
        self.prev_joint_pos = np.array(joint_pos_rad)
        return np.rad2deg(joint_pos_rad)

    def _get_default_posture(self):
        """获取所有关节的中间位置作为默认姿态"""
        posture = []
        for joint_name in self.joint_names:
            try:
                lower, upper = self.solver.get_joint_limits(joint_name)
                # 检查是不是无限旋转关节 (-inf)
                if lower < -100 or upper > 100: 
                    posture.append(0.0)
                else:
                    posture.append((lower + upper) / 2.0)
            except:
                posture.append(0.0)
        return np.array(posture, dtype=np.float64)