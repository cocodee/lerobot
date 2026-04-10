#!/usr/bin/env python

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

from lerobot.teleoperators.keyboard import KeyboardJointIKTeleop, KeyboardJointIKTeleopConfig
from lerobot.teleoperators.keyboard import teleop_keyboard as teleop_keyboard_module


class _FakeKinematics:
    def __init__(self, urdf_path, target_frame_name, joint_names):
        self.joint_names = joint_names

    def forward_kinematics(self, joint_pos_deg):
        pose = np.eye(4)
        pose[:3, 3] = np.array([0.1, 0.2, 0.3])
        return pose

    def inverse_kinematics(self, current_joint_pos, desired_ee_pose):
        updated = np.array(current_joint_pos, dtype=np.float64)
        updated[: len(updated) - 1] = updated[: len(updated) - 1] + 1.0
        return updated


def _make_config(output_mode="degrees"):
    return KeyboardJointIKTeleopConfig(
        id="keyboard_joint",
        urdf_path="/tmp/fake.urdf",
        output_mode=output_mode,
    )


def test_keyboard_joint_ik_action_features():
    teleop = KeyboardJointIKTeleop(_make_config())

    assert teleop.action_features == {
        "shoulder_pan.pos": float,
        "shoulder_lift.pos": float,
        "elbow_flex.pos": float,
        "wrist_flex.pos": float,
        "wrist_roll.pos": float,
        "gripper.pos": float,
    }


def test_keyboard_joint_ik_generates_joint_space_action(monkeypatch):
    monkeypatch.setattr(teleop_keyboard_module, "RobotKinematics", _FakeKinematics)
    monkeypatch.setattr(KeyboardJointIKTeleop, "is_connected", property(lambda self: True))

    teleop = KeyboardJointIKTeleop(_make_config())
    teleop.current_pressed = {"w": True, "u": True, "m": True}

    action = teleop.get_action()

    assert set(action) == set(teleop.action_features)
    assert action["shoulder_pan.pos"] == 1.0
    assert action["gripper.pos"] == 55.0


def test_keyboard_joint_ik_normalized_output(monkeypatch):
    monkeypatch.setattr(teleop_keyboard_module, "RobotKinematics", _FakeKinematics)
    monkeypatch.setattr(KeyboardJointIKTeleop, "is_connected", property(lambda self: True))

    teleop = KeyboardJointIKTeleop(_make_config(output_mode="normalized"))
    teleop.current_pressed = {"w": True}

    action = teleop.get_action()

    assert all(-100.0 <= action[f"{joint}.pos"] <= 100.0 for joint in teleop.config.joint_names)
    assert action["gripper.pos"] == 50.0
