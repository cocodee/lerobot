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

"""
Velocity and acceleration limiting for safe robot control.

Maintains a history of states to compute velocities and accelerations,
then clamps target positions to ensure limits are respected.
"""

import logging
import numpy as np
from collections import deque
from typing import Dict, Tuple, Optional, List
from dataclasses import dataclass

from .safety_config import VelocityLimits, AccelerationLimits

logger = logging.getLogger(__name__)


@dataclass
class StateHistory:
    """Stores historical state for velocity/acceleration computation."""
    timestamp: float
    joint_positions: np.ndarray
    ee_position: Optional[np.ndarray] = None
    ee_orientation: Optional[np.ndarray] = None


class VelocityLimiter:
    """
    Limits joint velocities and accelerations to safe ranges.

    Maintains a history of states to compute velocities and accelerations,
    then clamps target positions to ensure limits are respected.

    Args:
        velocity_limits: Velocity limit configuration.
        acceleration_limits: Acceleration limit configuration.
        joint_names: List of joint names for logging.
        control_frequency: Control loop frequency in Hz.
        history_length: Number of previous states to keep for velocity/acceleration computation.

    Example:
        ```python
        limiter = VelocityLimiter(
            velocity_limits=VelocityLimits(max_joint_velocity=30.0),
            acceleration_limits=AccelerationLimits(max_joint_acceleration=50.0),
            joint_names=["joint1", "joint2", ...],
            control_frequency=30.0
        )
        limited_positions, info = limiter.limit_joint_action(
            target_positions=np.array([10.0, 20.0, ...]),
            current_positions=np.array([0.0, 0.0, ...]),
            current_time=time.time()
        )
        ```
    """

    def __init__(
        self,
        velocity_limits: VelocityLimits,
        acceleration_limits: AccelerationLimits,
        joint_names: List[str],
        control_frequency: float,
        history_length: int = 3,
    ):
        self.velocity_limits = velocity_limits
        self.acceleration_limits = acceleration_limits
        self.joint_names = joint_names
        self.dt = 1.0 / control_frequency
        self.history_length = history_length

        # State history for velocity/acceleration computation
        self.state_history: deque[StateHistory] = deque(maxlen=history_length)

        # Previous velocity for acceleration computation
        self.previous_velocity: Optional[np.ndarray] = None
        self.previous_velocity_time: Optional[float] = None

        logger.info(
            f"VelocityLimiter initialized: max_vel={velocity_limits.max_joint_velocity} deg/s, "
            f"max_accel={acceleration_limits.max_joint_acceleration} deg/s²"
        )

    def limit_joint_action(
        self,
        target_positions: np.ndarray,
        current_positions: np.ndarray,
        current_time: float,
    ) -> Tuple[np.ndarray, Dict[str, any]]:
        """
        Limit target positions based on velocity and acceleration constraints.

        Args:
            target_positions: Desired joint positions in degrees.
            current_positions: Current joint positions in degrees.
            current_time: Current timestamp for velocity computation.

        Returns:
            A tuple of (limited_positions, info):
                - limited_positions: Clamped positions respecting limits
                - info: Dictionary with details about what was limited
        """
        info = {
            'velocity_limited': False,
            'acceleration_limited': False,
            'joints_limited': [],
            'original_max_velocity': 0.0,
            'limited_max_velocity': 0.0,
        }

        # Compute current velocity
        current_velocity = self._compute_velocity(current_positions, current_time)

        # Step 1: Limit based on velocity constraints
        limited_positions = target_positions.copy()

        # Compute desired velocity
        desired_velocity = (target_positions - current_positions) / self.dt

        # Clamp desired velocity
        max_vel = self.velocity_limits.max_joint_velocity
        velocity_clamped = np.clip(desired_velocity, -max_vel, max_vel)

        if not np.allclose(desired_velocity, velocity_clamped, atol=1e-6):
            info['velocity_limited'] = True
            info['original_max_velocity'] = float(np.max(np.abs(desired_velocity)))
            info['limited_max_velocity'] = float(np.max(np.abs(velocity_clamped)))

            # Find which joints were limited
            limited_indices = np.where(np.abs(desired_velocity) > max_vel)[0]
            info['joints_limited'] = [self.joint_names[i] for i in limited_indices if i < len(self.joint_names)]

            # Recompute positions with clamped velocity
            limited_positions = current_positions + velocity_clamped * self.dt

        # Step 2: Limit based on acceleration constraints
        if self.previous_velocity is not None:
            desired_acceleration = (velocity_clamped - self.previous_velocity) / self.dt

            # Clamp acceleration
            max_accel = self.acceleration_limits.max_joint_acceleration
            acceleration_clamped = np.clip(
                desired_acceleration, -max_accel, max_accel
            )

            if not np.allclose(desired_acceleration, acceleration_clamped, atol=1e-6):
                info['acceleration_limited'] = True

                # Recompute velocity with acceleration limit
                new_velocity = self.previous_velocity + acceleration_clamped * self.dt

                # Apply velocity limit again after acceleration adjustment
                new_velocity = np.clip(new_velocity, -max_vel, max_vel)

                limited_positions = current_positions + new_velocity * self.dt

        # Update state history
        self.state_history.append(StateHistory(
            timestamp=current_time,
            joint_positions=current_positions.copy()
        ))

        # Update previous velocity
        self.previous_velocity = current_velocity
        self.previous_velocity_time = current_time

        # Log if limiting occurred
        if info['velocity_limited']:
            logger.debug(
                f"Velocity limited: {info['joints_limited']} "
                f"({info['original_max_velocity']:.1f} → {info['limited_max_velocity']:.1f} deg/s)"
            )

        return limited_positions, info

    def limit_ee_action(
        self,
        target_ee_pose: np.ndarray,
        current_ee_pose: np.ndarray,
        current_time: float,
    ) -> Tuple[np.ndarray, Dict[str, any]]:
        """
        Limit end-effector motion based on velocity/acceleration constraints.

        Args:
            target_ee_pose: Desired EE pose as 4x4 transformation matrix.
            current_ee_pose: Current EE pose as 4x4 transformation matrix.
            current_time: Current timestamp.

        Returns:
            A tuple of (limited_pose, info):
                - limited_pose: Clamped pose respecting limits
                - info: Dictionary with details about what was limited
        """
        info = {
            'velocity_limited': False,
            'acceleration_limited': False,
        }

        limited_pose = target_ee_pose.copy()

        # Extract positions
        current_pos = current_ee_pose[:3, 3]
        target_pos = target_ee_pose[:3, 3]

        # Compute desired velocity
        desired_vel = (target_pos - current_pos) / self.dt

        # Clamp linear velocity
        max_linear_vel = self.velocity_limits.max_ee_velocity
        vel_clamped = np.clip(
            desired_vel, -max_linear_vel, max_linear_vel
        )

        if not np.allclose(desired_vel, vel_clamped, atol=1e-6):
            info['velocity_limited'] = True
            limited_pose[:3, 3] = current_pos + vel_clamped * self.dt

        # TODO: Add rotation velocity/acceleration limiting
        # This requires converting quaternion differences to angular velocity

        return limited_pose, info

    def _compute_velocity(
        self,
        current_positions: np.ndarray,
        current_time: float,
    ) -> np.ndarray:
        """
        Compute velocity from state history.

        Args:
            current_positions: Current joint positions.
            current_time: Current timestamp.

        Returns:
            Computed velocity in deg/s.
        """
        if len(self.state_history) < 1:
            return np.zeros_like(current_positions)

        prev_state = self.state_history[-1]
        dt = current_time - prev_state.timestamp

        if dt < 1e-6:
            return np.zeros_like(current_positions)

        return (current_positions - prev_state.joint_positions) / dt

    def reset(self):
        """Reset velocity limiter state.

        Call this when starting a new episode or after a long pause.
        """
        self.state_history.clear()
        self.previous_velocity = None
        self.previous_velocity_time = None
        logger.debug("VelocityLimiter reset")

    def get_state(self) -> dict:
        """Get current state of the velocity limiter.

        Returns:
            Dictionary with current velocity and state history length.
        """
        return {
            'current_velocity': self.previous_velocity.tolist() if self.previous_velocity is not None else [],
            'history_length': len(self.state_history),
        }
