# lerobot/src/lerobot/robots/sim_robot/sim_robot.py
import logging
import time
import math
import pybullet as p
import pybullet_data
import numpy as np
from functools import cached_property
from typing import Any, Dict, Tuple

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.robots import Robot
from .config_sim_robot import SimRobotConfig
from ..utils import ensure_safe_goal_position

logger = logging.getLogger(__name__)

class SimRobot(Robot):
    config_class = SimRobotConfig
    name = "sim_robot"

    def __init__(self, config: SimRobotConfig):
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self._is_calibrated = True  # 仿真环境默认已校准
        self.simulator = None

        # 为了对应真机修改
        self.joint_names = [
            "shoulder_roll_left", "shoulder_lift_left", "elbow_roll_left", 
            "elbow_flex_left", "wrist_roll_left", "gripper_flex_left", "gripper_left",
        ]

        self.robot2sim = {
            "left_arm_joint_1": "shoulder_roll_left", "left_arm_joint_2":"shoulder_lift_left",
            "left_arm_joint_3": "elbow_roll_left", "left_arm_joint_4": "elbow_flex_left",
            "left_arm_joint_5": "wrist_roll_left", "left_arm_joint_6": "gripper_flex_left",
            "left_arm_joint_7": "gripper_left",
            "right_arm_joint_1": "shoulder_roll_right", "right_arm_joint_2":"shoulder_lift_right",
            "right_arm_joint_3": "elbow_roll_right", "right_arm_joint_4": "elbow_flex_right",
            "right_arm_joint_5": "wrist_roll_right", "right_arm_joint_6": "gripper_flex_right",
            "right_arm_joint_7": "gripper_right",
            "trunk_joint_1":"body_roll", "trunk_joint_2":"waist_flex"
        }

        # 相机配置（从config转换）
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motors_ft(self) -> dict[str, type]:
        """电机特征定义（遵循仓库格式）"""
        return {f"{motor}.pos": float for motor in self.joint_names}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        """相机特征定义"""
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) 
            for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        """观测特征集合"""
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        """动作特征集合"""
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self._is_connected and self.simulator is not None

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        # 初始化仿真环境
        self.simulator = self._create_simulator()
        self._is_connected = True
        logger.info(f"{self} connected to PyBullet simulator")

        # 连接相机（仿真环境中无需实际硬件连接）
        for cam in self.cameras.values():
            cam.connect()

    def _create_simulator(self) -> Any:
        """创建并初始化仿真环境"""
        from .simulator import Simulator  # 导入用户提供的仿真器类
        return Simulator(
            headless=self.config.headless,
        )

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

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        # 获取关节状态
        joint_positions, _ = self.simulator.get_joint_states()
        obs_dict = {
            f"{name}.pos": math.degrees(joint_positions[i])
            # f"{name}.pos": joint_positions[i]
            for i, name in enumerate(self.joint_names)
        }
        # print("obs_action: ", obs_dict)
        print("obs_action rad: ", joint_positions)

        # 获取相机图像
        images = self.simulator.get_camera_images()
        for cam_name, img in images.items():
            obs_dict[cam_name] = img

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        # import pdb; pdb.set_trace()
        # 机器人关节名称和仿真名称映射
        if "left_arm_joint_1.pos" in action:
            new_action = {}
            for key, val in action.items():
                name = self.robot2sim[key.split(".")[0]]
                new_action[f"{name}.pos"] = val
            action = new_action
        
        # import pdb; pdb.set_trace()
        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}
        # Cap goal position when too far away from present position.
        # /!\ Slower fps expected due to reading from the follower.
        if self.config.max_relative_target is not None:
            joint_positions, _ = self.simulator.get_joint_states()
            present_pos = {
                f"{name}": math.degrees(joint_positions[i])
                # f"{name}.pos": joint_positions[i]
                for i, name in enumerate(self.joint_names)
            }
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)
            action_array = np.array([
                math.radians(goal_pos[name]) for name in self.joint_names
            ])
            print("using actiong:", np.array([
                goal_pos[name] for name in self.joint_names
            ]))

        else:
            # 转换动作格式（从字典到数组）
            action_array = np.array([
                math.radians(action[f"{name}.pos"]) for name in self.joint_names
                # action[f"{name}.pos"] for name in self.joint_names
            ])

            print("using actiong:", np.array([
                action[f"{name}.pos"] for name in self.joint_names
            ]))
        # import pdb; pdb.set_trace()
        # 执行仿真步骤
        # _, _, done, _ = self.simulator.step(action_array)
        next_obs = self.simulator.step(action_array)
        # if done:
        #     logger.info("Simulation episode completed")
        logger.info("Simulation episode completed")
       
        # print("using act rad:",action_array)

        return action

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        self.simulator.close()
        self.simulator = None
        self._is_connected = False

        # 断开相机连接
        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected from simulator")

    def get_joint_names(self) -> List[str]:
        """返回所有关节的名称列表"""
        return self.joint_names

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
            name: math.degrees(joint_positions[i])
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
        self.send_action(action)
    def get_present_current(self) -> Dict[str, float]:
        """
        获取当前关节电流 (仿真中通常返回 0 或力矩)。
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")
        
        # 仿真通常不模拟电流，或者可以通过 getJointStates 的第4个返回值获取 torque
        # 这里为了简单起见，返回 0.0，或者你可以修改 Simulator 获取 torque
        return {name: 0.0 for name in self.joint_names}

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
        gripper_name = "gripper_left" # 默认假设
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
