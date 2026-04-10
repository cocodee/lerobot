#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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
import os
import sys
import time
from queue import Queue
from typing import Any

import numpy as np

from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.model.kinematics import RobotKinematics

from ..teleoperator import Teleoperator
from .configuration_keyboard import (
    KeyboardEndEffectorTeleopConfig,
    KeyboardJointIKTeleopConfig,
    KeyboardTeleopConfig,
)
import logging


logger =  logging.getLogger(__name__)
PYNPUT_AVAILABLE = True
try:
    if ("DISPLAY" not in os.environ) and ("linux" in sys.platform):
        logging.info("No DISPLAY set. Skipping pynput import.")
        raise ImportError("pynput blocked intentionally due to no display.")

    from pynput import keyboard
except ImportError:
    keyboard = None
    PYNPUT_AVAILABLE = False
except Exception as e:
    keyboard = None
    PYNPUT_AVAILABLE = False
    logging.info(f"Could not import pynput: {e}")


class KeyboardTeleop(Teleoperator):
    """
    Teleop class to use keyboard inputs for control.
    """

    config_class = KeyboardTeleopConfig
    name = "keyboard"

    def __init__(self, config: KeyboardTeleopConfig):
        super().__init__(config)
        self.config = config
        self.robot_type = config.type

        self.event_queue = Queue()
        self.current_pressed = {}
        self.listener = None
        self.logs = {}

    @property
    def action_features(self) -> dict:
        return {
            "dtype": "float32",
            "shape": (len(self.arm),),
            "names": {"motors": list(self.arm.motors)},
        }

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return PYNPUT_AVAILABLE and isinstance(self.listener, keyboard.Listener) and self.listener.is_alive()

    @property
    def is_calibrated(self) -> bool:
        pass

    def connect(self) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(
                "Keyboard is already connected. Do not run `robot.connect()` twice."
            )

        if PYNPUT_AVAILABLE:
            logging.info("pynput is available - enabling local keyboard listener.")
            self.listener = keyboard.Listener(
                on_press=self._on_press,
                on_release=self._on_release,
            )
            self.listener.start()
        else:
            logging.info("pynput not available - skipping local keyboard listener.")
            self.listener = None

    def calibrate(self) -> None:
        pass

    def _on_press(self, key):
        if hasattr(key, "char"):
            self.event_queue.put((key.char, True))

    def _on_release(self, key):
        if hasattr(key, "char"):
            self.event_queue.put((key.char, False))
        if key == keyboard.Key.esc:
            logging.info("ESC pressed, disconnecting.")
            self.disconnect()

    def _drain_pressed_keys(self):
        while not self.event_queue.empty():
            key_char, is_pressed = self.event_queue.get_nowait()
            self.current_pressed[key_char] = is_pressed

    def configure(self):
        pass

    def get_action(self) -> dict[str, Any]:
        before_read_t = time.perf_counter()

        if not self.is_connected:
            raise DeviceNotConnectedError(
                "KeyboardTeleop is not connected. You need to run `connect()` before `get_action()`."
            )

        self._drain_pressed_keys()

        # Generate action based on current key states
        action = {key for key, val in self.current_pressed.items() if val}
        self.logs["read_pos_dt_s"] = time.perf_counter() - before_read_t

        return dict.fromkeys(action, None)

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(
                "KeyboardTeleop is not connected. You need to run `robot.connect()` before `disconnect()`."
            )
        if self.listener is not None:
            self.listener.stop()
    def reset(self) -> None:
        """Resets the teleoperator state."""
        # 清空当前按下的键
        self.current_pressed.clear()
        
        # 清空事件队列 (使用 mutex 确保线程安全)
        with self.event_queue.mutex:
            self.event_queue.queue.clear()
            
        self.logs.clear()
        logger.info(f"{self.name} teleoperator state reset.")

class KeyboardEndEffectorTeleop(KeyboardTeleop):
    """
    Teleop class to use keyboard inputs for end effector control.
    Designed to be used with the `So100FollowerEndEffector` robot.
    """

    config_class = KeyboardEndEffectorTeleopConfig
    name = "keyboard_ee"

    def __init__(self, config: KeyboardEndEffectorTeleopConfig):
        super().__init__(config)
        self.config = config
        self.misc_keys_queue = Queue()
        self.continuous_keys = {
                #keyboard.Key.up,
                #keyboard.Key.down,
                #keyboard.Key.left,
                #keyboard.Key.right,
                keyboard.Key.shift,
                keyboard.Key.shift_l,
                keyboard.Key.shift_r,
                keyboard.Key.ctrl_l,
                keyboard.Key.ctrl_r,
                # 如果有 WASD 控制，也要加在这里，注意要用字符
                'w', 'a', 's', 'd' 
        }
        self.pending_releases = {}
        self.DEBOUNCE_THRESHOLD = 0.05 

    def _drain_pressed_keys(self):
        """
        混合处理模式：
        1. 对 self.continuous_keys 里的键：使用防抖逻辑（解决 Linux 长按闪烁问题）。
        2. 对其他键：使用原始逻辑（直接赋值，无延迟）。
        """
        curr_time = time.perf_counter()

        # 1. 消费事件队列
        while not self.event_queue.empty():
            key, is_pressed = self.event_queue.get_nowait()
            
            # 判断是否为需要长按防抖的连续键
            if key in self.continuous_keys:
                # === 新方式 (Debounce) ===
                if is_pressed:
                    self.current_pressed[key] = True
                    if key in self.pending_releases:
                        del self.pending_releases[key]
                else:
                    # 释放时不立即移除，而是放入待定区
                    self.pending_releases[key] = curr_time
            else:
                # === 原始方式 (Direct) ===
                # 不需要防抖，直接反映当前状态
                # 任何待定的释放操作如果涉及此键，直接清除，以最新状态为准             
                if is_pressed:
                    self.current_pressed[key] = True
                else:
                    self.current_pressed[key] = False

        # 2. 处理防抖键的延迟释放
        keys_to_remove = []
        for key, release_time in self.pending_releases.items():
            if curr_time - release_time > self.DEBOUNCE_THRESHOLD:
                if key in self.current_pressed:
                    del self.current_pressed[key]
                keys_to_remove.append(key)
        
        for key in keys_to_remove:
            del self.pending_releases[key]
    @property
    def action_features(self) -> dict:
        if self.config.use_gripper:
            return {
                "dtype": "float32",
                "shape": (4,),
                "names": {"delta_x": 0, "delta_y": 1, "delta_z": 2, "gripper": 3},
            }
        else:
            return {
                "dtype": "float32",
                "shape": (3,),
                "names": {"delta_x": 0, "delta_y": 1, "delta_z": 2},
            }

    def _on_press(self, key):
        if hasattr(key, "char"):
            key = key.char
            logger.info(f"Key pressed: {key}")  
        else:
            logger.info(f"Special Key pressed: {key}")          
        self.event_queue.put((key, True))

    def _on_release(self, key):
        if hasattr(key, "char"):
            key = key.char
            logger.info(f"Key released: {key}")
        else:
            logger.info(f"Special Key released: {key}")
        self.event_queue.put((key, False))

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(
                "KeyboardTeleop is not connected. You need to run `connect()` before `get_action()`."
            )

        self._drain_pressed_keys()
        delta_x = 0.0
        delta_y = 0.0
        delta_z = 0.0
        gripper_action = 1.0

        # Generate action based on current key states
        for key, val in self.current_pressed.items():
            logger.info(f"key: {key}, val: {val}")
            val = 1
            #if key == keyboard.Key.up:
            #    delta_y = -int(val)
            #    logger.info(f"delta_y: {delta_y}")
            #elif key == keyboard.Key.down:
            #    delta_y = int(val)
            #    logger.info(f"delta_y: {delta_y}")
            #elif key == keyboard.Key.left:
            #    delta_x = int(val)
            #    logger.info(f"delta_x: {delta_x}")
            #elif key == keyboard.Key.right:
            #    delta_x = -int(val)
            #    logger.info(f"delta_x: {delta_x}")
            if key == "w":
                delta_y = -int(val)
                logger.info(f"delta_y: {delta_y}")
            elif key == "s":
                delta_y = int(val)
                logger.info(f"delta_y: {delta_y}")
            elif key == "a":
                delta_x = int(val)
                logger.info(f"delta_x: {delta_x}")
            elif key == "d":
                delta_x = -int(val)
                logger.info(f"delta_x: {delta_x}")
            elif key == keyboard.Key.shift:
                delta_z = -int(val)
            elif key == keyboard.Key.shift_r:
                delta_z = int(val)
            elif key == keyboard.Key.ctrl_r:
                # Gripper actions are expected to be between 0 (close), 1 (stay), 2 (open)
                gripper_action = int(val) + 1
            elif key == keyboard.Key.ctrl_l:
                gripper_action = int(val) - 1
            else:
                # If the key is pressed, add it to the misc_keys_queue
                # this will record key presses that are not part of the delta_x, delta_y, delta_z
                # this is useful for retrieving other events like interventions for RL, episode success, etc.
                self.misc_keys_queue.put(key)

        for key in list(self.current_pressed.keys()):
            if not key in self.continuous_keys:
                self.current_pressed.pop(key)

        action_dict = {
            "delta_x": delta_x,
            "delta_y": delta_y,
            "delta_z": delta_z,
        }

        if self.config.use_gripper:
            action_dict["gripper"] = gripper_action

        return action_dict

    def reset(self) -> None:
        """Resets the teleoperator state including debounce buffers and misc queues."""
        # 调用父类 reset 清除 current_pressed 和 event_queue
        super().reset()
        
        # 清除杂项按键队列 (如用于记录成功的键)
        with self.misc_keys_queue.mutex:
            self.misc_keys_queue.queue.clear()
            
        # 清除防抖等待字典
        self.pending_releases.clear()
        
        logger.info(f"{self.name} extended state reset.")


def _rotation_matrix_x(angle_deg: float) -> np.ndarray:
    angle_rad = np.deg2rad(angle_deg)
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)


def _rotation_matrix_y(angle_deg: float) -> np.ndarray:
    angle_rad = np.deg2rad(angle_deg)
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


def _rotation_matrix_z(angle_deg: float) -> np.ndarray:
    angle_rad = np.deg2rad(angle_deg)
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


class KeyboardJointIKTeleop(KeyboardEndEffectorTeleop):
    """
    Keyboard teleop that maintains an end-effector pose internally and outputs joint-space actions.

    Unlike `KeyboardEndEffectorTeleop`, this teleoperator solves IK locally and exposes the same
    joint-space action keys as leader-arm teleoperators, which allows it to be used inside
    `SwitchableTeleoperator`.
    """

    config_class = KeyboardJointIKTeleopConfig
    name = "keyboard_joint_ik"

    def __init__(self, config: KeyboardJointIKTeleopConfig):
        super().__init__(config)
        self.config = config
        self.command_by_key = dict(config.key_bindings)
        self.continuous_keys = set(self.command_by_key)
        self.misc_keys_queue = Queue()
        self._kinematics = None
        self.current_joint_pos_deg: np.ndarray | None = None
        self.current_ee_pose: np.ndarray | None = None

    @property
    def action_features(self) -> dict[str, type]:
        action_joint_names = [*self.config.joint_names, self.config.gripper_joint_name]
        return {f"{joint}.pos": float for joint in action_joint_names}

    def _ensure_kinematics(self) -> None:
        if self._kinematics is not None:
            return

        self._kinematics = RobotKinematics(
            urdf_path=self.config.urdf_path,
            target_frame_name=self.config.target_frame_name,
            joint_names=self.config.joint_names,
        )

        self.current_joint_pos_deg = np.array(
            [self.config.initial_joint_positions[joint] for joint in [*self.config.joint_names, self.config.gripper_joint_name]],
            dtype=np.float64,
        )
        self.current_ee_pose = self._kinematics.forward_kinematics(self.current_joint_pos_deg)

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(
                "KeyboardJointIKTeleop is not connected. You need to run `connect()` before `get_action()`."
            )

        self._ensure_kinematics()
        self._drain_pressed_keys()

        assert self.current_joint_pos_deg is not None
        assert self.current_ee_pose is not None

        delta_translation = np.zeros(3, dtype=np.float64)
        delta_rotation = np.eye(3, dtype=np.float64)
        gripper_delta = 0.0

        for key in list(self.current_pressed.keys()):
            if not self.current_pressed[key]:
                continue

            command = self.command_by_key.get(key)
            if command is None:
                self.misc_keys_queue.put(key)
                continue

            if command == "translate_x_positive":
                delta_translation[0] += self.config.end_effector_step_sizes["x"]
            elif command == "translate_x_negative":
                delta_translation[0] -= self.config.end_effector_step_sizes["x"]
            elif command == "translate_y_positive":
                delta_translation[1] += self.config.end_effector_step_sizes["y"]
            elif command == "translate_y_negative":
                delta_translation[1] -= self.config.end_effector_step_sizes["y"]
            elif command == "translate_z_positive":
                delta_translation[2] += self.config.end_effector_step_sizes["z"]
            elif command == "translate_z_negative":
                delta_translation[2] -= self.config.end_effector_step_sizes["z"]
            elif command == "rotate_roll_positive":
                delta_rotation = delta_rotation @ _rotation_matrix_x(self.config.rotation_step_sizes_deg["roll"])
            elif command == "rotate_roll_negative":
                delta_rotation = delta_rotation @ _rotation_matrix_x(-self.config.rotation_step_sizes_deg["roll"])
            elif command == "rotate_pitch_positive":
                delta_rotation = delta_rotation @ _rotation_matrix_y(self.config.rotation_step_sizes_deg["pitch"])
            elif command == "rotate_pitch_negative":
                delta_rotation = delta_rotation @ _rotation_matrix_y(-self.config.rotation_step_sizes_deg["pitch"])
            elif command == "rotate_yaw_positive":
                delta_rotation = delta_rotation @ _rotation_matrix_z(self.config.rotation_step_sizes_deg["yaw"])
            elif command == "rotate_yaw_negative":
                delta_rotation = delta_rotation @ _rotation_matrix_z(-self.config.rotation_step_sizes_deg["yaw"])
            elif command == "gripper_open":
                gripper_delta += self.config.gripper_step_size
            elif command == "gripper_close":
                gripper_delta -= self.config.gripper_step_size
            else:
                self.misc_keys_queue.put(key)

        desired_ee_pose = self.current_ee_pose.copy()
        desired_ee_pose[:3, 3] = self.current_ee_pose[:3, 3] + delta_translation
        desired_ee_pose[:3, 3] = np.clip(
            desired_ee_pose[:3, 3],
            self.config.end_effector_bounds["min"],
            self.config.end_effector_bounds["max"],
        )
        desired_ee_pose[:3, :3] = self.current_ee_pose[:3, :3] @ delta_rotation

        new_joint_pos_deg = self._kinematics.inverse_kinematics(self.current_joint_pos_deg, desired_ee_pose)
        gripper_index = len(self.config.joint_names)
        new_joint_pos_deg = np.array(new_joint_pos_deg, dtype=np.float64)
        new_joint_pos_deg[gripper_index] = np.clip(
            self.current_joint_pos_deg[gripper_index] + gripper_delta,
            self.config.gripper_bounds["min"],
            self.config.gripper_bounds["max"],
        )

        self.current_joint_pos_deg = new_joint_pos_deg
        self.current_ee_pose = desired_ee_pose

        for key in list(self.current_pressed.keys()):
            if key not in self.continuous_keys:
                self.current_pressed.pop(key)

        return self._format_joint_action(self.current_joint_pos_deg)

    def _format_joint_action(self, joint_pos_deg: np.ndarray) -> dict[str, float]:
        arm_joint_values = joint_pos_deg[: len(self.config.joint_names)]
        gripper_value = float(joint_pos_deg[len(self.config.joint_names)])

        if self.config.output_mode == "degrees":
            arm_action = {
                f"{joint}.pos": float(value) for joint, value in zip(self.config.joint_names, arm_joint_values, strict=True)
            }
        else:
            arm_action = {}
            for joint, value in zip(self.config.joint_names, arm_joint_values, strict=True):
                joint_bounds = self.config.joint_degree_bounds[joint]
                min_deg = joint_bounds["min"]
                max_deg = joint_bounds["max"]
                bounded_value = float(np.clip(value, min_deg, max_deg))
                normalized = (((bounded_value - min_deg) / (max_deg - min_deg)) * 200.0) - 100.0
                arm_action[f"{joint}.pos"] = normalized

        arm_action[f"{self.config.gripper_joint_name}.pos"] = gripper_value
        return arm_action

    def reset(self) -> None:
        super().reset()
        self.current_joint_pos_deg = None
        self.current_ee_pose = None
