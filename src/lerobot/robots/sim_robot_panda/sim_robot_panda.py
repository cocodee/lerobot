import logging
import math
import numpy as np
from typing import Any, Dict, List
from functools import cached_property

from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.robots import Robot
from ..sim_robot.config_sim_robot import SimRobotPandaConfig
from ..utils import ensure_safe_goal_position

logger = logging.getLogger(__name__)

class SimRobotPanda(Robot):
    config_class = SimRobotPandaConfig
    name = "sim_robot_panda"

    def __init__(self, config: SimRobotPandaConfig):
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self._is_calibrated = True
        self.simulator = None

        # Franka Panda 关节定义 (MuJoCo XML 中的关节名称)
        # 通常为 joint1 到 joint7，以及 finger_joint1, finger_joint2
        self.joint_names = [
            "joint1", "joint2", "joint3", "joint4", 
            "joint5", "joint6", "joint7", 
        ]
        
        # 定义动作空间中对应的名称（这里简化，假设左右指动作为同一个 gripper 信号）
        # 这里的 key 是 lerobot 数据集中使用的名称，value 是仿真中的名称
        self.robot2sim = {
            "left_arm_joint_1": "joint1",
            "left_arm_joint_2": "joint2",
            "left_arm_joint_3": "joint3",
            "left_arm_joint_4": "joint4",
            "left_arm_joint_5": "joint5",
            "left_arm_joint_6": "joint6",
            "left_arm_joint_7": "joint7",
        }
        self.sim2robot = {v: k for k, v in self.robot2sim.items()}

        # 简单的相机配置占位符
        self.cameras = {} 

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{name}.pos": float for name in self.robot2sim.keys()}

    @property
    def is_connected(self) -> bool:
        return self._is_connected and self.simulator is not None

    def connect(self) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.simulator = self._create_simulator()
        self._is_connected = True
        logger.info(f"{self} connected to MuJoCo simulator")

    def _create_simulator(self) -> Any:
        from .simulator import MujocoSimulator
        return MujocoSimulator(
            config=self.config,
            headless=self.config.headless,
        )

    def disconnect(self) -> None:
        if not self.is_connected:
            return
        self.simulator.close()
        self.simulator = None
        self._is_connected = False

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        # 获取仿真器数据 (positions: rad, velocities: rad/s)
        joint_positions, _ = self.simulator.get_joint_states()
        
        obs_dict = {}
        # 映射仿真关节到机器人关节名称
        # 注意：Sim 返回的数组顺序对应 self.joint_names
        for i, sim_name in enumerate(self.joint_names):
            if sim_name in self.sim2robot:
                robot_name = self.sim2robot[sim_name]
                # 转换为角度 (degrees) 以保持与原代码逻辑一致，或者保持弧度取决于你的训练pipeline
                # 这里假设需要转换为度数
                obs_dict[f"{robot_name}.pos"] = math.degrees(joint_positions[i])
            elif sim_name == "finger_joint2":
                # 夹爪通常是对称的，不需要单独记录 finger2，或者可以取平均
                pass

        # 获取图像
        images = self.simulator.get_camera_images()
        for cam_name, img in images.items():
            obs_dict[cam_name] = img

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        # 1. 解析 Action 字典到仿真器需要的数组
        # 初始化目标位置数组，长度为所有仿真关节
        target_positions = np.zeros(len(self.joint_names))
        
        # 获取当前位置作为默认值 (防止未指定的关节归零)
        current_pos, _ = self.simulator.get_joint_states()
        target_positions[:] = current_pos

        for key, val in action.items():
            # key 例如 "panda_joint1.pos"
            clean_key = key.split(".")[0]
            if clean_key in self.robot2sim:
                sim_name = self.robot2sim[clean_key]
                # 找到该关节在 joint_names 中的索引
                idx = self.joint_names.index(sim_name)
                # 将度数转回弧度
                target_positions[idx] = math.radians(val)
                
                # 特殊处理夹爪：如果是 gripper，同时控制两个手指
                if clean_key == "panda_gripper":
                    # 假设 finger_joint2 是 finger_joint1 的镜像或从动
                    idx2 = self.joint_names.index("finger_joint2")
                    target_positions[idx2] = math.radians(val)

        # 2. 执行仿真步进
        self.simulator.step(target_positions)
        
        return action
    
    @property
    def is_calibrated(self) -> bool:
        return self._is_calibrated

    def calibrate(self) -> None:
        """仿真环境无需实际校准"""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")
        self._is_calibrated = True
        logger.info(f"{self} calibration skipped (simulation)")

    def configure(self) -> None:
        """配置仿真环境参数"""
        pass

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        """观测特征集合"""
        return {**self._motors_ft, **self._cameras_ft}