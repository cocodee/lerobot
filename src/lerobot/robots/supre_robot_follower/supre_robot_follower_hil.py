# !/usr/bin/env python

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

import logging
import time
from typing import Any,List,Dict

import numpy as np

from lerobot.cameras import make_cameras_from_configs
from lerobot.errors import DeviceNotConnectedError
from lerobot.model.kinematics import RobotKinematics

from .supre_robot_follower import SupreRobotFollower
from .supre_robot_follower_config import SupreRobotFollowerHilConfig

logger = logging.getLogger(__name__)


class SupreRobotFollowerHil(SupreRobotFollower):
    """
    SO100Follower robot with end-effector space control.

    This robot inherits from SO100Follower but transforms actions from
    end-effector space to joint space before sending them to the motors.
    """

    config_class = SupreRobotFollowerHilConfig
    name = "supre_robot_follower_hil"

    def __init__(self, config: SupreRobotFollowerHilConfig):
        super().__init__(config)

        self.cameras = make_cameras_from_configs(config.cameras)

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
        )

        # Store the bounds for end-effector position
        self.end_effector_bounds = self.config.end_effector_bounds

        self.current_ee_pos = None
        self.current_joint_pos = None

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
                if "gripper" not in action:
                    action["gripper"] = [1.0]
                action = np.append(delta_ee, action["gripper"])
            else:
                logger.warning(
                    f"Expected action keys 'delta_x', 'delta_y', 'delta_z', got {list(action.keys())}"
                )
                action = np.zeros(4, dtype=np.float32)

        if self.current_joint_pos is None:
            # Read current joint positions
            #TODO:获取当前关节位置
            current_joint_pos = self.bus.sync_read("Present_Position")
            self.current_joint_pos = np.array([current_joint_pos[name] for name in self.bus.motors])

        # Calculate current end-effector position using forward kinematics
        if self.current_ee_pos is None:
            self.current_ee_pos = self.kinematics.forward_kinematics(self.current_joint_pos)

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
            self.current_joint_pos, desired_ee_pos
        )

        # Create joint space action dictionary
        #TODO: joint names
        joint_action = {
            f"{key}.pos": target_joint_values_in_degrees[i] for i, key in enumerate(self.bus.motors.keys())
        }

        # Handle gripper separately if included in action
        # Gripper delta action is in the range 0 - 2,
        # We need to shift the action to the range -1, 1 so that we can expand it to -Max_gripper_pos, Max_gripper_pos
        #TODO:gripper
        joint_action["gripper.pos"] = np.clip(
            self.current_joint_pos[-1] + (action[-1] - 1) * self.config.max_gripper_pos,
            5,
            self.config.max_gripper_pos,
        )

        self.current_ee_pos = desired_ee_pos.copy()
        self.current_joint_pos = target_joint_values_in_degrees.copy()
        self.current_joint_pos[-1] = joint_action["gripper.pos"]

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
        return self.observation_joint_names

    def get_present_position(self) -> Dict[str, float]:
        """
        获取当前关节位置。
        返回格式: {'joint_name': position, ...}
        """
        if not self.is_connected:
            raise RuntimeError("Robot is not connected.")
        # 复用已有的 get_current_position 方法
        return self.get_current_position()

    def write_goal_position(self, target_position: Dict[str, float]) -> None:
        """
        向机器人写入目标位置。
        Args:
            target_position: 包含 {'joint_name': target_pos} 的字典
        """
        if not self.is_connected:
            raise RuntimeError("Robot is not connected.")

        # 将字典转换为按照 joint_order 排序的列表，因为硬件管理器通常接受列表
        try:
            target_list = [target_position[name] for name in self.observation_joint_names]
            self._hardware_manager.write(target_list)
        except KeyError as e:
            logger.error(f"Target position dict is missing joint: {e}")
            raise ValueError(f"Missing joint {e} in target_position")

    def get_present_current(self) -> Dict[str, float]:
        """
        获取当前关节电流（或力/力矩，取决于硬件实现）。
        返回格式: {'joint_name': current, ...}
        """
        if not self.is_connected:
            raise RuntimeError("Robot is not connected.")
        
        # read() 返回 (positions, forces)
        _, forces = self._hardware_manager.read()
        
        # 将力列表打包成字典
        return {
            name: force 
            for name, force in zip(self.observation_joint_names, forces)
        }

    def set_enable_torque(self, enable: bool) -> None:
        """
        启用或禁用关节扭矩。
        注意：这取决于 SupreRobotHardwareManager 是否暴露了相应的接口。
        """
        if not self.is_connected:
            return

        # 尝试调用硬件管理器上的方法（如果存在）
        if hasattr(self._hardware_manager, "set_enable_torque"):
            self._hardware_manager.set_enable_torque(enable)
        else:
            # 如果硬件管理器没有显式的方法，记录警告
            logger.warning(f"Hardware manager does not support explicit torque control (set_enable_torque={enable}).")

    def get_gripper_position(self) -> float:
        """获取夹爪的当前位置"""
        positions = self.get_present_position()

        gripper_joint_name = self.config.gripper_joint_name
        if gripper_joint_name in positions:
            return positions[gripper_joint_name]
        
        logger.warning("Gripper joint not found in present positions.")
        return 0.0

    def get_max_gripper_position(self) -> float:
        """从配置中获取最大夹爪位置"""
        # 确保 SupreRobotFollowerConfig 中定义了 max_gripper_pos
        return getattr(self.config, "max_gripper_pos", 100.0) # 默认值防止崩溃

    def get_urdf_path(self) -> str:
        """获取 URDF 文件路径"""
        # 确保 SupreRobotFollowerConfig 中定义了 urdf_path
        return getattr(self.config, "urdf_path", None)
