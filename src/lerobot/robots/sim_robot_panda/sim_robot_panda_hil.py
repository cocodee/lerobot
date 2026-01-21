# lerobot/src/lerobot/robots/sim_robot/sim_robot_panda_hil.py

import logging
import time
import math
import numpy as np
import mujoco
from functools import cached_property
from typing import Any, Dict, List, Optional
from scipy.spatial.transform import Rotation as R

from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.model.be_kinematics import BeRobotKinematics
from ..sim_robot.config_sim_robot import SimRobotPandaHilConfig
from .sim_robot_panda import SimRobotPanda  # 确保这里导入的是 MuJoCo 版本的 SimRobot
from .webxr_intent_translator import WebXRIntentTranslator
from .differential_ik_wrapper import DifferentialIKWrapper
import traceback

logger = logging.getLogger(__name__)


# Franka Panda 在 URDF 中的标准关节名称
PANDA_URDF_JOINT_NAMES = [
    "joint1", "joint2", "joint3", 
    "joint4", "joint5", "joint6", "joint7"
]

class SimRobotPandaHil(SimRobotPanda):
    config_class = SimRobotPandaHilConfig
    name = "sim_robot_panda_hil"

    def __init__(self, config: SimRobotPandaHilConfig):
        super().__init__(config)
        self.config = config

        # 1. 初始化运动学模块 (IK/FK)
        # 注意: 即使仿真用的是 XML，IK 求解器通常需要 URDF
        if self.config.urdf_path is None:
            raise ValueError(
                "urdf_path must be provided for kinematics (IK). "
                "Ensure config.urdf_path points to 'panda.urdf'."
            )

        self.kinematics = BeRobotKinematics(
            urdf_path=self.config.urdf_path,
            target_frame_name=self.config.target_frame_name, # 通常是 "panda_link8" 或 "panda_hand"
            joint_names=PANDA_URDF_JOINT_NAMES,
        )
        self.diff_ik = DifferentialIKWrapper(self.kinematics)

        self.end_effector_bounds = self.config.end_effector_bounds
        self.current_ee_pos = None
        self.current_joint_pos = None

        # 3. 初始化 WebXR 意图翻译器 
        matrix = np.array([
            [ 0,  1, 0], # Robot X 来自 WebXR Y
            [0,  0,  -1], # Robot Y 来自 -WebXR X
            [ -1,  0,  0]  # Robot Z 来自  WebXR Y
        ])
        ##matrix = np.array([
        ##    [1, 0,  0],
        ##    [0, 0, -1],
        ##    [0, 1,  0]
        ##])
        mapping_matrix = np.array([
            [0, 1, 0],  # Row 0
            [1, 0, 0],  # Row 1
            [0, 0, 1]   # Row 2
        ])

        # 确保这是一个合法的旋转矩阵（行列式为1）
        # print(np.linalg.det(mapping_matrix)) # 应该是 1.0

        r_fix = R.from_matrix(mapping_matrix)
        self.webxr_translator = WebXRIntentTranslator(xr_to_robot_matrix=matrix,axis_map_rotation=r_fix)
        
        # 2. 定义仿真器中的关节名称 (对应 MuJoCo XML)
        # MuJoCo Menagerie 的 panda.xml 通常使用 joint1...joint7
        self.sim_joint_names = [
            "joint1", "joint2", "joint3", "joint4", 
            "joint5", "joint6", "joint7"
        ]
        
        
        # 关节方向修正 (如果 XML 和 URDF 定义方向不一致，在此修改，Panda 通常一致)
        self.joint_direction = np.ones(7) 

    @property
    def action_features(self) -> dict[str, Any]:
        return {
            "dtype": "float32",
            "shape": (4,),
            "names": {
                "delta_x": 0, "delta_y": 1, "delta_z": 2, 
                "delta_qx": 3, "delta_qy": 4, "delta_qz": 5, "delta_qw": 6, 
            },
        }

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """
        接收末端执行器 (EE) 的 Delta 动作，通过 IK 转换为关节角度，发送给 MuJoCo 仿真器。
        或者接收 WebXR 原始数据，使用 WebXRIntentTranslator 转换为目标 EE 位姿。
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # --- 1. 获取当前状态 (Feedback) ---
        # 获取仿真器中的当前关节角度
        sim_joint_state = self.get_present_joint_state() # 返回 {sim_name: degrees}

        # 转换为 URDF 需要的顺序和单位 (FK/IK 需要度数或弧度，Lerobot Kinematics 默认通常是度数)
        self.current_joint_pos = np.array([sim_joint_state[name] for name in self.get_joint_names()])

        # --- 2. 正运动学 (FK) 获取当前 EE 位姿 ---
        # self.current_ee_pos 是一个 4x4 齐次矩阵
        try:
            self.current_ee_pos = self.kinematics.forward_kinematics(self.current_joint_pos)
            if self.current_ee_pos is None:
                logger.error("forward_kinematics returned None")
                return {}
        except Exception as e:
            logger.error(f"forward_kinematics failed: {e}")
            traceback.print_exc()
            return {}

        # --- 3. 解析 Action ---
        logger.info(f"send_action: {action}")
        # 检查是否为 WebXR 格式 (包含 p, q, g, m, type)
        if isinstance(action, dict) and action.get("type") == "webxr":
            # 使用 WebXRIntentTranslator 计算目标 EE 位姿
            desired_ee_pos = self.webxr_translator.update(
                frame=action,
                T_current=self.current_ee_pos
            )

            # 如果翻译器返回 None (IDLE 模式或无有效转换)，保持当前位置
            if desired_ee_pos is None:
                desired_ee_pos = self.current_ee_pos.copy()

            # 获取夹爪值
            gripper_val = float(action.get("g", 1.0))

            logger.info(f"send_action WebXR mode: {action.get('m', 'IDLE')}, desired_ee_pos: {desired_ee_pos}")

        else:
            # --- 传统的 Delta EE 动作处理 ---
            delta_quat = None
            delta_ee = np.zeros(3)
            gripper_val = 1.0

            if isinstance(action, dict):
                if all(k in action for k in ["delta_x", "delta_y", "delta_z"]):
                    delta_ee = np.array([
                        action["delta_x"] * self.config.end_effector_step_sizes["x"],
                        action["delta_y"] * self.config.end_effector_step_sizes["y"],
                        action["delta_z"] * self.config.end_effector_step_sizes["z"],
                    ], dtype=np.float32)

                    # 提取四元数 (x, y, z, w)
                    if "delta_qw" in action:
                        delta_quat = np.array([
                            action["delta_qx"], action["delta_qy"],
                            action["delta_qz"], action["delta_qw"]
                        ], dtype=np.float32)

                    if "gripper" in action:
                        gripper_val = action["gripper"]
                else:
                    logger.warning(f"Invalid action keys: {list(action.keys())}, keeping current position")
                    desired_ee_pos = self.current_ee_pos.copy()

            logger.info(f"send_action Action: {action}, delta_ee: {delta_ee}")

            # --- 4. 计算目标 EE 位姿 ---
            desired_ee_pos = np.eye(4)

            # 旋转计算 (使用 Scipy 替代 PyBullet)
            current_rot_mat = self.current_ee_pos[:3, :3]

            if delta_quat is not None:
                # Scipy Rotation 输入顺序是 (x, y, z, w)
                r_delta = R.from_quat(delta_quat)
                delta_rot_mat = r_delta.as_matrix()

                # R_new = R_delta * R_curr (或者根据控制逻辑 R_curr * R_delta)
                # 这里沿用 SimRobotHil 逻辑：左乘 delta
                new_rot_mat = delta_rot_mat @ current_rot_mat
                desired_ee_pos[:3, :3] = new_rot_mat
            else:
                desired_ee_pos[:3, :3] = current_rot_mat

            # 位置计算
            desired_ee_pos[:3, 3] = self.current_ee_pos[:3, 3] + delta_ee

        # 边界截断
        if self.end_effector_bounds is not None:
            desired_ee_pos[:3, 3] = np.clip(
                desired_ee_pos[:3, 3],
                self.end_effector_bounds["min"],
                self.end_effector_bounds["max"],
            )

        # --- 5. 逆运动学 (IK) ---
        # 计算目标关节角度 (Degrees)
        #target_joint_values_deg = self.kinematics.inverse_kinematics(
        #    self.current_joint_pos, desired_ee_pos
        #)
        target_joint_values_deg = self.diff_ik.step(
            self.current_joint_pos, desired_ee_pos
        )

        # --- 6. 构造发送给 SimRobot 的 Joint Action ---
        joint_action = {}
        
        # 映射回仿真器名称
        for i, urdf_name in enumerate(PANDA_URDF_JOINT_NAMES):
            name = self.sim2robot[urdf_name]
            # SimRobot 父类接受 f"{sim_name}.pos"
            joint_action[f"{name}.pos"] = target_joint_values_deg[i]

        # --- 7. 处理夹爪 ---
        # Panda 夹爪范围通常是 0 (闭合) 到 0.04m (单指打开)，总宽度 0.08m
        # 假设 action['gripper'] 是 -1 (闭) 到 1 (开) 或者 0..1
        # config.max_gripper_pos 应该设为 0.04 (单指行程)
        
        # 映射逻辑: action(0~1) -> width(0 ~ max)
        # 如果 SimRobotConfig 中 gripper_joint_name 是 "panda_gripper"
        # 我们需要将其映射到仿真器的 "finger_joint1" (和 "finger_joint2")
        
        # 简单的线性映射，假设传入的是 [0, 1] 范围
        max_width = getattr(self.config, "max_gripper_pos", 0.04)
        target_gripper_pos = gripper_val * max_width # 0 ~ 0.04
        
        # MuJoCo 中我们控制 finger_joint1 (主)
        joint_action["finger_joint1.pos"] = target_gripper_pos
        # 如果需要显式控制第二个手指
        joint_action["finger_joint2.pos"] = target_gripper_pos
        
        logger.info(f"currrent_joint_pos: {self.current_joint_pos}")
        logger.info(f"Target Joints (deg): {target_joint_values_deg}")
        logger.info(f"joint_action: {joint_action}")
        # Debug Drawing (MuJoCo版本)
        if hasattr(self.simulator, "viewer") and self.simulator.viewer:
            # MuJoCo 绘图比较复杂，这里仅简单打印，或者你可以扩展 Simulator 类
            # self._mujoco_debug_draw(desired_ee_pos)
            pass

        # 发送给父类 (SimRobot) 执行底层 step
        return super().send_action(joint_action)
    def get_joint_names(self) -> List[str]:
        """返回 URDF 中定义的关节名称"""
        return ["left_arm_joint_1",
                "left_arm_joint_2",
                "left_arm_joint_3",
                "left_arm_joint_4",
                "left_arm_joint_5",
                "left_arm_joint_6",
                "left_arm_joint_7"]

    def _mujoco_debug_draw(self, frame_matrix):
        """
        MuJoCo 可视化 Debug (占位符)
        在 MuJoCo 中动态画线需要操作 viewer.user_scn，这需要修改 render loop。
        为简单起见，这里仅做日志记录。
        """
        pos = frame_matrix[:3, 3]
        logger.debug(f"Target EE: {pos}")
        
        # 如果你想实现，需要在 Simulator 类中暴露 mjv_scene 接口
        pass

    def get_present_joint_state(self) -> Dict[str, float]:
        """
        获取当前关节状态，并进行单位转换和重命名。
        返回格式: {'panda_joint1': degrees, ..., 'panda_finger_joint1': meters}
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")
        
        # 1. 获取仿真器原始数据 
        # positions: 旋转关节为 Rad, 线性关节为 Meters
        joint_positions_raw, _ = self.simulator.get_joint_states()
        
        # 2. 获取仿真器中的关节名称列表 (用于索引匹配)
        # 假设 simulator.joint_names 顺序与 joint_positions_raw 一致
        sim_names = self.simulator.joint_names
        
        state_dict = {}
        
        for i, sim_name in enumerate(sim_names):
            # 检查该仿真关节是否有对应的机器人映射名称
            if sim_name in self.sim2robot:
                robot_name = self.sim2robot[sim_name]
                raw_val = joint_positions_raw[i]
                
                # 3. 单位转换逻辑
                if "finger" in sim_name or "gripper" in sim_name:
                    # --- 夹爪 (Prismatic Joint) ---
                    # 保持单位为米 (Meters)，不要转为角度
                    state_dict[robot_name] = raw_val
                else:
                    # --- 机械臂 (Revolute Joint) ---
                    # 这里的 joint1-7 是旋转关节，通常需要转为角度 (Degrees)
                    # 只有转为角度，IK 求解器 (RobotKinematics) 才能正确计算
                    state_dict[robot_name] = math.degrees(raw_val)
        
        # 4. 可选：如果需要在日志中查看当前状态
        logger.info(f"Present Joint State: {state_dict}")
        
        return state_dict    
    
    def get_urdf_path(self) -> str:
        """获取 URDF 文件路径"""
        return getattr(self.config, "urdf_path", None)
    
    def get_present_position(self) -> Dict[str, float]:
        """
        获取当前关节位置。
        返回格式: {'joint_name': position_in_degrees, ...}
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")
        
        # simulator.get_joint_states 返回的是弧度
        joint_positions, _ = self.simulator.get_joint_states()
        
        return {
            self.sim2robot[name]: math.degrees(joint_positions[i])*self.joint_direction[i]
            for i, name in enumerate(self.joint_names)
        }
    
    def write_goal_position(self, target_position: Dict[str, float]) -> None:
        """
        向机器人写入目标位置。
        通过调用 send_action 实现，包含重映射、安全检查和仿真步进。
        Args:
            target_position: 包含 {'joint_name': target_pos_degrees} 的字典
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        # 构造符合 send_action 预期的字典 (确保带 .pos 后缀)
        action = {}
        for name, value in target_position.items():
            # 如果键名已经是 'joint.pos' 格式则保持，否则添加后缀
            key = name if name.endswith(".pos") else f"{name}.pos"
            action[key] = value

        # 直接调用 send_action，复用其内部的名称映射(robot2sim)和单位转换逻辑
        super().send_action(action)    

    def reset(self):
        self.current_ee_pos = None
        self.current_joint_pos = None
        self.debug_accumulated_pose = None        

    def get_present_current(self) -> Dict[str, float]:
        """
        获取当前关节电流 (仿真中通常返回 0 或力矩)。
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")
        
        # 仿真通常不模拟电流，或者可以通过 getJointStates 的第4个返回值获取 torque
        # 这里为了简单起见，返回 0.0，或者你可以修改 Simulator 获取 torque
        return {self.sim2robot[name]: 0.0 for name in self.joint_names}        
    
    def get_urdf_joint_names(self) -> List[str]:
        """获取 URDF 中的关节名称"""
        return PANDA_URDF_JOINT_NAMES    
    
    def get_gripper_position(self) -> float:
        """获取夹爪的当前位置"""
        positions = self.get_present_position()
        
        # 假设 gripper 关节名称包含 'gripper'
        # 根据 self.joint_names 查找
        gripper_name = "left_arm_joint_7" # 默认假设
        for name in self.joint_names:
            if "gripper" in name and "flex" not in name: # 排除 flex 关节
                gripper_name = name
                break
        
        return positions.get(gripper_name, 0.0)    