# lerobot/src/lerobot/robots/sim_robot/sim_robot.py
import logging
import time
import math
import pybullet as p
import pybullet_data
import numpy as np
from functools import cached_property
from typing import Any, Dict, Tuple,List
from scipy.spatial.transform import Rotation as R

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.robots import Robot
from lerobot.model.be_kinematics import BeRobotKinematics
from .config_sim_robot import SimRobotConfig, SimRobotHilConfig
from .sim_robot import SimRobot
from ..sim_robot_panda.differential_ik_wrapper import DifferentialIKWrapper
from ..sim_robot_panda.webxr_intent_translator import WebXRIntentTranslator
from ..utils import ensure_safe_goal_position

import traceback
logger = logging.getLogger(__name__)

URDF_JOINT_NAMES = [
    "left_arm_joint0",
    "left_arm_joint1",
    "left_arm_joint2",
    "left_arm_joint3",
    "left_arm_joint4",
    "left_arm_joint5",
]

URDF_JOINT_NAMES = [
            "arml_joint", "arml_joint1", "arml_joint2", 
            "arml_joint3", "arml_joint4", "arml_joint5",
        ]
class SimRobotHil(SimRobot):
    config_class = SimRobotHilConfig
    name = "sim_robot_hil"

    def __init__(self, config: SimRobotHilConfig):
        super().__init__(config)
        self.config = config
        # Initialize the kinematics module for the supre robot
        if self.config.urdf_path is None:
            raise ValueError(
                "urdf_path must be provided in the configuration for end-effector control. "
                "Please set urdf_path in your SupreRobotFollowerEndEffectorConfig."
            )

        self.kinematics = BeRobotKinematics(
            urdf_path=self.config.urdf_path,
            target_frame_name=self.config.target_frame_name,
            joint_names=URDF_JOINT_NAMES,
        )
        self.diff_ik = DifferentialIKWrapper(self.kinematics)

        # Initialize WebXR Intent Translator
        matrix = np.array([
            [ -1,  0, 0], 
            [ 0,  1,  0], 
            [ 0,  0,  -1]  
        ])
        mapping_matrix = np.array([
            [-1, 0, 0],  # Row 0   r->p,p->y,y->r
            [0, 1, 0],  # Row 1   r->p,
            [0, 0, -1]   # Row 2
        ])
        r_fix = R.from_matrix(mapping_matrix)
        self.webxr_translator = WebXRIntentTranslator(xr_to_robot_matrix=matrix, axis_map_rotation=r_fix)

        # Store the bounds for end-effector position
        self.end_effector_bounds = self.config.end_effector_bounds

        self.current_ee_pos = None
        self.current_joint_pos = None
        self.debug_accumulated_pose = None
        # 为了对应真机修改

    @property
    def action_features(self) -> dict[str, Any]:
        """
        Define action features for end-effector control.
        Returns dictionary with dtype, shape, and names.
        """
        return {
            "dtype": "float32",
            "shape": (4,),
            "names": {"delta_x": 0, "delta_y": 1, "delta_z": 2, "delta_qx":3, "delta_qy":4,
                      "delta_qz":5, "delta_qw":6, "gripper": 7},
        }

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """
        Transform action from end-effector space to joint space and send to motors.
        Supports WebXR format actions and traditional delta EE actions.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # --- 1. 获取当前状态 (Feedback) ---
        # 获取仿真器中的当前关节角度
        sim_joint_state = self.get_present_joint_state()  # 返回 {sim_name: degrees}

        # 转换为 URDF 需要的顺序和单位
        self.current_joint_pos = np.array([sim_joint_state[name] for name in self.get_joint_names()])

        # --- 2. 正运动学 (FK) 获取当前 EE 位姿 ---
        try:
            self.current_ee_pos = self.kinematics.forward_kinematics(self.current_joint_pos[:-1])
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

                # R_new = R_delta * R_curr
                new_rot_mat = delta_rot_mat @ current_rot_mat
                desired_ee_pos[:3, :3] = new_rot_mat
            else:
                desired_ee_pos[:3, :3] = current_rot_mat

            # 位置计算
            desired_ee_pos[:3, 3] = self.current_ee_pos[:3, 3] + delta_ee

        # --- 5. 边界截断 ---
        if self.end_effector_bounds is not None:
            desired_ee_pos[:3, 3] = np.clip(
                desired_ee_pos[:3, 3],
                self.end_effector_bounds["min"],
                self.end_effector_bounds["max"],
            )

        # --- 6. 逆运动学 (IK) ---
        # 使用 DifferentialIKWrapper 进行微分 IK 计算
        target_joint_values_deg = self.diff_ik.step(
            self.current_joint_pos[:-1], desired_ee_pos,joint_task_weight=0.001,target_joints={"arml_joint3": 90},
        )

        # --- 7. 构造发送给 SimRobot 的 Joint Action ---
        joint_action = {}

        # 映射回仿真器名称
        for i, joint_name in enumerate(self.joint_names[:-1]):
            name = self.sim2robot[joint_name]
            # SimRobot 父类接受 f"{sim_name}.pos"
            joint_action[f"{name}.pos"] = target_joint_values_deg[i] * self.joint_direction[i]

        # --- 8. 处理夹爪 ---
        gripper_pos_name = self.config.gripper_joint_name + '.pos'
        joint_action[gripper_pos_name] = np.clip(
            self.current_joint_pos[-1] + (gripper_val - 1) * self.config.max_gripper_pos,
            5,
            self.config.max_gripper_pos,
        )

        logger.info(f"current_joint_pos: {self.current_joint_pos}")
        logger.info(f"Target Joints (deg): {target_joint_values_deg}")
        logger.info(f"joint_action: {joint_action}")

        # Debug Drawing
        self._debug_draw_frame(desired_ee_pos, label="Target", life_time=0.5)
        self._debug_draw_frame(self.current_ee_pos, label="Current", life_time=0.5)

        # 更新当前状态
        self.current_ee_pos = desired_ee_pos.copy()
        self.current_joint_pos = np.append(
            target_joint_values_deg.copy(),
            joint_action[gripper_pos_name]
        )

        # 发送给父类 (SimRobot) 执行底层 step
        return super().send_action(joint_action)        

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Read arm position
        return super().get_observation()
    def reset(self):
        self.current_ee_pos = None
        self.current_joint_pos = None
        self.debug_accumulated_pose = None
    def get_joint_names(self) -> List[str]:
        """返回所有关节的名称列表"""
        return ["left_arm_joint_1",
                "left_arm_joint_2",
                "left_arm_joint_3",
                "left_arm_joint_4",
                "left_arm_joint_5",
                "left_arm_joint_6",
                "left_arm_joint_7"]

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

    def get_present_joint_state(self) -> Dict[str, float]:
        """
        获取当前关节位置。
        返回格式: {'joint_name': position_in_degrees, ...}
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")
        
        # simulator.get_joint_states 返回的是弧度
        joint_positions, _ = self.simulator.get_joint_states()
        
        logger.info(f"get_joint_positions: {joint_positions}")
        return {
            self.sim2robot[name]: math.degrees(joint_positions[i])
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
    def get_present_current(self) -> Dict[str, float]:
        """
        获取当前关节电流 (仿真中通常返回 0 或力矩)。
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")
        
        # 仿真通常不模拟电流，或者可以通过 getJointStates 的第4个返回值获取 torque
        # 这里为了简单起见，返回 0.0，或者你可以修改 Simulator 获取 torque
        return {self.sim2robot[name]: 0.0 for name in self.joint_names}

    def set_enable_torque(self, enable: bool) -> None:
        """
        仿真中通常总是启用扭矩，此为兼容性空方法。
        """
        pass

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

    def get_max_gripper_position(self) -> float:
        """从配置中获取最大夹爪位置"""
        return getattr(self.config, "max_gripper_pos", 1.0) # 默认值

    def get_urdf_path(self) -> str:
        """获取 URDF 文件路径"""
        return getattr(self.config, "urdf_path", None)


    def _debug_draw_frame(self, frame_matrix, label="frame", life_time=0.1, line_width=2):
        """
        修正后的绘图方法：
        在 PyBullet 中画出一个 4x4 矩阵代表的坐标系。
        红色=X轴, 绿色=Y轴, 蓝色=Z轴
        """
        origin = frame_matrix[:3, 3]       # 提取平移向量
        rotation = frame_matrix[:3, :3]    # 提取旋转矩阵
        
        # 轴的长度 (例如 10cm)
        length = 0.1
        
        # 定义局部坐标系的轴向量
        x_axis_local = np.array([length, 0, 0])
        y_axis_local = np.array([0, length, 0])
        z_axis_local = np.array([0, 0, length])
        
        # 【关键修改】：将局部轴向量旋转到目标坐标系的方向
        # 使用矩阵乘法: R * v
        x_axis_rotated = rotation @ x_axis_local
        y_axis_rotated = rotation @ y_axis_local
        z_axis_rotated = rotation @ z_axis_local
        
        robot_id = self.simulator.robot_id
        base_link_index = -1
        
        # 画线：从原点指向旋转后的轴端点
        # 红色 X轴
        p.addUserDebugLine(origin, origin + x_axis_rotated, [1, 0, 0], 
                           lifeTime=life_time, lineWidth=line_width,
                           parentObjectUniqueId=robot_id, parentLinkIndex=base_link_index)
        # 绿色 Y轴
        p.addUserDebugLine(origin, origin + y_axis_rotated, [0, 1, 0], 
                           lifeTime=life_time, lineWidth=line_width,
                           parentObjectUniqueId=robot_id, parentLinkIndex=base_link_index)
        # 蓝色 Z轴
        p.addUserDebugLine(origin, origin + z_axis_rotated, [0, 0, 1], 
                           lifeTime=life_time, lineWidth=line_width,
                           parentObjectUniqueId=robot_id, parentLinkIndex=base_link_index)
        
        # 显示文字标签
        p.addUserDebugText(label, origin, [0, 0, 0], 
                           lifeTime=life_time,
                           parentObjectUniqueId=robot_id, parentLinkIndex=base_link_index)

    def base_pos_2_world_pos(self, ee_pose_in_base) -> np.ndarray:
        """
        将机器人基座位置转换为世界坐标系下的位置
        """
        sim_base_pos, sim_base_orn = p.getBasePositionAndOrientation(self.simulator.robot_id)
        logger.info(f"sim_base_pos: {sim_base_pos}, sim_base_orn: {sim_base_orn}")
        # 将四元数转换为 3x3 旋转矩阵
        base_rot_matrix = np.reshape(p.getMatrixFromQuaternion(sim_base_orn), (3, 3))
    
        # 构建 4x4 齐次变换矩阵 T_base_to_world
        T_base_to_world = np.eye(4)
        T_base_to_world[:3, :3] = base_rot_matrix # 填入旋转
        T_base_to_world[:3, 3] = sim_base_pos     # 填入平移
    
        # 3. 矩阵乘法：将相对坐标转换为世界坐标
        # ee_pose_in_world = T_base_to_world * ee_pose_in_base
        ee_pose_in_world = T_base_to_world @ ee_pose_in_base
        return ee_pose_in_world
    
    def get_urdf_joint_names(self) -> List[str]:
        """获取 URDF 中的关节名称"""
        return URDF_JOINT_NAMES