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

from dataclasses import dataclass, field

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("keyboard")
@dataclass
class KeyboardTeleopConfig(TeleoperatorConfig):
    # TODO(Steven): Consider setting in here the keys that we want to capture/listen
    mock: bool = False


@TeleoperatorConfig.register_subclass("keyboard_ee")
@dataclass
class KeyboardEndEffectorTeleopConfig(KeyboardTeleopConfig):
    use_gripper: bool = True


@TeleoperatorConfig.register_subclass("keyboard_joint_ik")
@dataclass
class KeyboardJointIKTeleopConfig(KeyboardTeleopConfig):
    urdf_path: str | None = None
    target_frame_name: str = "gripper_frame_link"
    joint_names: list[str] = field(
        default_factory=lambda: [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
        ]
    )
    gripper_joint_name: str = "gripper"
    initial_joint_positions: dict[str, float] = field(
        default_factory=lambda: {
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
            "gripper": 50.0,
        }
    )
    output_mode: str = "degrees"
    joint_degree_bounds: dict[str, dict[str, float]] = field(
        default_factory=lambda: {
            "shoulder_pan": {"min": -180.0, "max": 180.0},
            "shoulder_lift": {"min": -180.0, "max": 180.0},
            "elbow_flex": {"min": -180.0, "max": 180.0},
            "wrist_flex": {"min": -180.0, "max": 180.0},
            "wrist_roll": {"min": -180.0, "max": 180.0},
        }
    )
    end_effector_bounds: dict[str, list[float]] = field(
        default_factory=lambda: {
            "min": [-1.0, -1.0, -1.0],
            "max": [1.0, 1.0, 1.0],
        }
    )
    end_effector_step_sizes: dict[str, float] = field(
        default_factory=lambda: {
            "x": 0.01,
            "y": 0.01,
            "z": 0.01,
        }
    )
    rotation_step_sizes_deg: dict[str, float] = field(
        default_factory=lambda: {
            "roll": 5.0,
            "pitch": 5.0,
            "yaw": 5.0,
        }
    )
    gripper_step_size: float = 5.0
    gripper_bounds: dict[str, float] = field(default_factory=lambda: {"min": 0.0, "max": 100.0})
    key_bindings: dict[str, str] = field(
        default_factory=lambda: {
            "w": "translate_y_negative",
            "s": "translate_y_positive",
            "a": "translate_x_positive",
            "d": "translate_x_negative",
            "r": "translate_z_positive",
            "f": "translate_z_negative",
            "u": "rotate_roll_negative",
            "o": "rotate_roll_positive",
            "i": "rotate_pitch_positive",
            "k": "rotate_pitch_negative",
            "j": "rotate_yaw_positive",
            "l": "rotate_yaw_negative",
            "n": "gripper_close",
            "m": "gripper_open",
        }
    )

    def __post_init__(self):
        if self.urdf_path is None:
            raise ValueError("urdf_path must be provided for keyboard_joint_ik teleoperation.")

        if self.output_mode not in {"degrees", "normalized"}:
            raise ValueError("output_mode must be either 'degrees' or 'normalized'.")

        missing_joint_positions = [
            joint for joint in [*self.joint_names, self.gripper_joint_name] if joint not in self.initial_joint_positions
        ]
        if missing_joint_positions:
            raise ValueError(f"Missing initial_joint_positions for joints: {missing_joint_positions}")

        if self.output_mode == "normalized":
            missing_joint_bounds = [joint for joint in self.joint_names if joint not in self.joint_degree_bounds]
            if missing_joint_bounds:
                raise ValueError(f"Missing joint_degree_bounds for joints: {missing_joint_bounds}")
