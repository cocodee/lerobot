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
Safety configuration dataclasses for robot teleoperation.

Provides configurable parameters for collision detection, velocity/acceleration
limiting, and emergency stop functionality.
"""

from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class VelocityLimits:
    """Velocity limits for robot joints and end-effector.

    Attributes:
        max_joint_velocity: Maximum joint velocity in degrees/second.
        max_ee_velocity: Maximum end-effector linear velocity in m/s.
        max_ee_angular_velocity: Maximum end-effector angular velocity in rad/s.
    """

    max_joint_velocity: float = 30.0  # deg/s
    max_ee_velocity: float = 0.5  # m/s
    max_ee_angular_velocity: float = 1.0  # rad/s


@dataclass
class AccelerationLimits:
    """Acceleration limits for robot joints and end-effector.

    Attributes:
        max_joint_acceleration: Maximum joint acceleration in degrees/second².
        max_ee_acceleration: Maximum end-effector linear acceleration in m/s².
        max_ee_angular_acceleration: Maximum end-effector angular acceleration in rad/s².
    """

    max_joint_acceleration: float = 50.0  # deg/s²
    max_ee_acceleration: float = 1.0  # m/s²
    max_ee_angular_acceleration: float = 2.0  # rad/s²


@dataclass
class CollisionConfig:
    """Configuration for collision detection.

    Attributes:
        enabled: Whether collision detection is enabled.
        collision_threshold: Distance in meters to consider a collision (default: 5cm).
        check_self_collision: Whether to check for self-collisions.
        check_ground_collision: Whether to check for ground collisions.
        excluded_link_pairs: List of (link1, link2) tuples to exclude from collision checking.
            Useful for adjacent links that naturally touch.
    """

    enabled: bool = True
    collision_threshold: float = 0.05  # meters
    check_self_collision: bool = True
    check_ground_collision: bool = True
    excluded_link_pairs: List[tuple] = field(default_factory=list)


@dataclass
class EmergencyStopConfig:
    """Configuration for emergency stop mechanism.

    Attributes:
        enabled: Whether emergency stop is enabled.
        force_threshold: Force threshold in Newtons to trigger emergency stop (if force sensing available).
        velocity_threshold: Velocity threshold in deg/s to trigger emergency stop.
        watchdog_timeout: Communication timeout in seconds before triggering emergency stop.
        auto_recovery: Whether to automatically attempt recovery after emergency stop.
            If False, manual reset is required.
    """

    enabled: bool = True
    force_threshold: float = 50.0  # Newtons
    velocity_threshold: float = 100.0  # deg/s
    watchdog_timeout: float = 0.5  # seconds
    auto_recovery: bool = False


@dataclass
class SafetyConfig:
    """Complete safety configuration for a robot.

    This is the main configuration class that encompasses all safety features.
    Add this to your robot config to enable safety validation.

    Example:
        ```python
        from lerobot.robots.safety import SafetyConfig

        config = SupreRobotFollowerConfig(
            # ... existing config ...
            safety=SafetyConfig(
                enabled=True,
                velocity_limits=VelocityLimits(max_joint_velocity=30.0),
                collision=CollisionConfig(enabled=True),
                emergency_stop=EmergencyStopConfig(enabled=True),
            )
        )
        ```

    Attributes:
        enabled: Whether the safety system is enabled.
        velocity_limits: Velocity limit configuration.
        acceleration_limits: Acceleration limit configuration.
        collision: Collision detection configuration.
        emergency_stop: Emergency stop configuration.
        control_frequency: Control loop frequency in Hz (used for velocity computation).
        history_length: Number of previous states to keep for velocity/acceleration computation.
    """

    enabled: bool = True
    velocity_limits: VelocityLimits = field(default_factory=VelocityLimits)
    acceleration_limits: AccelerationLimits = field(default_factory=AccelerationLimits)
    collision: CollisionConfig = field(default_factory=CollisionConfig)
    emergency_stop: EmergencyStopConfig = field(default_factory=EmergencyStopConfig)
    control_frequency: float = 30.0  # Hz
    history_length: int = 3
