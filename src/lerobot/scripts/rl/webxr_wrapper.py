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


"""
Robot Environment for LeRobot Manipulation Tasks

This module provides a comprehensive gym-compatible environment for robot manipulation
with support for:
- Multiple robot types (SO100, SO101, Koch and Moss)
- Human intervention via leader-follower control or gamepad

- End-effector and joint space control
- Image processing (cropping and resizing)

The environment is built using a composable wrapper pattern where each wrapper
adds specific functionality to the base RobotEnv.

Example:
    env = make_robot_env(cfg)
    obs, info = env.reset()
    action = policy.select_action(obs)
    obs, reward, terminated, truncated, info = env.step(action)
"""

import logging
import time
from collections import deque
from threading import Lock
from typing import Annotated, Any, Sequence

import gymnasium as gym
import numpy as np
import torch
import torchvision.transforms.functional as F  # noqa: N812

from lerobot.cameras import opencv  # noqa: F401
from lerobot.configs import parser
from lerobot.envs.configs import EnvConfig
from lerobot.envs.utils import preprocess_observation
from lerobot.model.kinematics import RobotKinematics
from lerobot.robots import (  # noqa: F401
    RobotConfig,
    make_robot_from_config,
    sim_robot,
)
from lerobot.teleoperators import (
    gamepad,  # noqa: F401
    keyboard,  # noqa: F401
    webxr,
    make_teleoperator_from_config,
)
from lerobot.teleoperators.gamepad.teleop_gamepad import GamepadTeleop
from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardEndEffectorTeleop
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.utils import log_say

logging.basicConfig(level=logging.INFO)

logger = logging.getLogger(__name__)

