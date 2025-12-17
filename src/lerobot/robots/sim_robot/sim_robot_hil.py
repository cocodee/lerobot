# lerobot/src/lerobot/robots/sim_robot/sim_robot.py
import logging
import time
import math
import pybullet as p
import pybullet_data
import numpy as np
from functools import cached_property
from typing import Any, Dict, Tuple,List

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.robots import Robot
from lerobot.model.kinematics import RobotKinematics
from .config_sim_robot import SimRobotConfig, SimRobotHilConfig
from .sim_robot import SimRobot
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

        self.kinematics = RobotKinematics(
            urdf_path=self.config.urdf_path,
            target_frame_name=self.config.target_frame_name,
            joint_names=URDF_JOINT_NAMES,
        )

        # Store the bounds for end-effector position
        self.end_effector_bounds = self.config.end_effector_bounds

        self.current_ee_pos = None
        self.current_joint_pos = None
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
            "names": {"delta_x": 0, "delta_y": 1, "delta_z": 2, "gripper": 3},
        }

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """
        Transform action from end-effector space to joint space and send to motors.

        Args:
            action: Dictionary with keys 'delta_x', 'delta_y', 'delta_z' for end-effector control
                   or a numpy array with [delta_x, delta_y, delta_z]

        Returns:
            The joint-space action that was sent to the motors
        """

        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Convert action to numpy array if not already
        if isinstance(action, dict):
            if all(k in action for k in ["delta_x", "delta_y", "delta_z"]):
                delta_ee = np.array(
                    [
                        action["delta_x"] * self.config.end_effector_step_sizes["x"],
                        action["delta_y"] * self.config.end_effector_step_sizes["y"],
                        action["delta_z"] * self.config.end_effector_step_sizes["z"],
                    ],
                    dtype=np.float32,
                )
                logger.info(f"delta_ee: {delta_ee}")
                if "gripper" not in action:
                    action["gripper"] = [1.0]
                action = np.append(delta_ee, action["gripper"])
            else:
                logger.warning(
                    f"Expected action keys 'delta_x', 'delta_y', 'delta_z', got {list(action.keys())}"
                )
                traceback.print_stack()
                action = np.zeros(4, dtype=np.float32)

        #if self.current_joint_pos is None:
        if True:
            # Read current joint positions
            #TODO:获取当前关节位置
            current_joint_pos = self.get_present_position()
            self.current_joint_pos = np.array([current_joint_pos[name] for name in self.get_joint_names()])

        # Calculate current end-effector position using forward kinematics
        #if self.current_ee_pos is None:
        if True:
            self.current_ee_pos = self.kinematics.forward_kinematics(self.current_joint_pos[:-1])

        # Set desired end-effector position by adding delta
        desired_ee_pos = np.eye(4)
        desired_ee_pos[:3, :3] = self.current_ee_pos[:3, :3]  # Keep orientation

        # Add delta to position and clip to bounds
        desired_ee_pos[:3, 3] = self.current_ee_pos[:3, 3] + action[:3]
        if self.end_effector_bounds is not None:
            desired_ee_pos[:3, 3] = np.clip(
                desired_ee_pos[:3, 3],
                self.end_effector_bounds["min"],
                self.end_effector_bounds["max"],
            )

        # Compute inverse kinematics to get joint positions
        target_joint_values_in_degrees = self.kinematics.inverse_kinematics(
            self.current_joint_pos[:-1], desired_ee_pos
        )

        # Create joint space action dictionary
        #TODO: joint names
        joint_action = {
            f"{key}.pos": target_joint_values_in_degrees[i] for i, key in enumerate(self.get_joint_names()[:-1])
        }

        # Handle gripper separately if included in action
        # Gripper delta action is in the range 0 - 2,
        # We need to shift the action to the range -1, 1 so that we can expand it to -Max_gripper_pos, Max_gripper_pos
        #TODO:gripper
        gripper_pos_name = self.config.gripper_joint_name+'.pos'
        joint_action[gripper_pos_name] = np.clip(
            self.current_joint_pos[-1] + (action[-1] - 1) * self.config.max_gripper_pos,
            5,
            self.config.max_gripper_pos,
        )

        logger.info(f"current_ee_pos: {self.current_ee_pos}")
        logger.info(f"desired_ee_pos: {desired_ee_pos}")
        logger.info(f"current_joint_pos: {self.current_joint_pos}")
        logger.info(f"target_joint_values_in_degrees: {target_joint_values_in_degrees}")

        #self._debug_draw_frame(desired_ee_pos, label="Target", life_time=0.1)
        
        # 2. 画出当前的实际位置 (Actual)
        #self._debug_draw_frame(self.current_ee_pos, label="Current", life_time=0.1)        
        self.current_ee_pos = desired_ee_pos.copy()
        self.current_joint_pos = np.append(
            target_joint_values_in_degrees.copy(), 
            joint_action[gripper_pos_name]
        )

        # Send joint space action to parent class
        return super().send_action(joint_action)        

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Read arm position
        return super().get_observation()
    def reset(self):
        self.current_ee_pos = None
        self.current_joint_pos = None
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
        在 PyBullet 中画出一个 4x4 矩阵代表的坐标系。
        红色=X轴, 绿色=Y轴, 蓝色=Z轴
        """
        origin = frame_matrix[:3, 3]
        rotation = frame_matrix[:3, :3]
        
        # 轴的长度 (例如 10cm)
        length = 0.1
        
        # 本地坐标系的轴
        x_axis = np.array([length, 0, 0])
        y_axis = np.array([0, length, 0])
        z_axis = np.array([0, 0, length])
        
        # 转换到世界坐标系: origin + R @ axis
        p.addUserDebugLine(origin, origin + rotation @ x_axis, [1, 0, 0], lifeTime=life_time, lineWidth=line_width)
        p.addUserDebugLine(origin, origin + rotation @ y_axis, [0, 1, 0], lifeTime=life_time, lineWidth=line_width)
        p.addUserDebugLine(origin, origin + rotation @ z_axis, [0, 0, 1], lifeTime=life_time, lineWidth=line_width)
        
        # 可选：显示文字标签
        p.addUserDebugText(label, origin, [0, 0, 0], lifeTime=life_time)