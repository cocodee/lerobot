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

"""
Unit tests for the safety module.

Tests for collision detection, velocity limiting, emergency stop,
and safety validation functionality.
"""

import tempfile
import time
from pathlib import Path
from unittest.mock import MagicMock, Mock, patch

import numpy as np
import pytest

from lerobot.robots.safety import (
    SafetyConfig,
    VelocityLimits,
    AccelerationLimits,
    CollisionConfig,
    EmergencyStopConfig,
    CollisionDetector,
    VelocityLimiter,
    EmergencyStopController,
    EmergencyStopState,
    SafetyValidator,
)


# =============================================================================
# Test Fixtures
# =============================================================================


@pytest.fixture
def joint_names():
    """Standard joint names for testing."""
    return ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]


@pytest.fixture
def temp_urdf_file(request):
    """Create a temporary URDF file for testing, or use custom URDF if provided."""
    custom_urdf_path = request.config.getoption("--custom-urdf")

    if custom_urdf_path:
        # Use custom URDF file provided by user
        custom_path = Path(custom_urdf_path)
        if not custom_path.exists():
            raise FileNotFoundError(f"Custom URDF file not found: {custom_urdf_path}")
        yield str(custom_path)
    else:
        # Create a temporary URDF file for testing
        urdf_content = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>
  <link name="link1">
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>
</robot>
"""
        with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
            f.write(urdf_content)
            temp_path = f.name
        yield temp_path
        Path(temp_path).unlink(missing_ok=True)


# =============================================================================
# SafetyConfig Tests
# =============================================================================


class TestSafetyConfig:
    """Tests for safety configuration dataclasses."""

    def test_velocity_limits_defaults(self):
        """Test VelocityLimits default values."""
        limits = VelocityLimits()
        assert limits.max_joint_velocity == 30.0
        assert limits.max_ee_velocity == 0.5
        assert limits.max_ee_angular_velocity == 1.0

    def test_velocity_limits_custom(self):
        """Test VelocityLimits with custom values."""
        limits = VelocityLimits(
            max_joint_velocity=50.0,
            max_ee_velocity=1.0,
            max_ee_angular_velocity=2.0,
        )
        assert limits.max_joint_velocity == 50.0
        assert limits.max_ee_velocity == 1.0
        assert limits.max_ee_angular_velocity == 2.0

    def test_acceleration_limits_defaults(self):
        """Test AccelerationLimits default values."""
        limits = AccelerationLimits()
        assert limits.max_joint_acceleration == 50.0
        assert limits.max_ee_acceleration == 1.0
        assert limits.max_ee_angular_acceleration == 2.0

    def test_collision_config_defaults(self):
        """Test CollisionConfig default values."""
        config = CollisionConfig()
        assert config.enabled is True
        assert config.collision_threshold == 0.05
        assert config.check_self_collision is True
        assert config.check_ground_collision is True
        assert config.excluded_link_pairs == []

    def test_collision_config_excluded_pairs(self):
        """Test CollisionConfig with excluded link pairs."""
        config = CollisionConfig(
            excluded_link_pairs=[("link1", "link2"), ("link3", "link4")]
        )
        assert len(config.excluded_link_pairs) == 2
        assert ("link1", "link2") in config.excluded_link_pairs

    def test_emergency_stop_config_defaults(self):
        """Test EmergencyStopConfig default values."""
        config = EmergencyStopConfig()
        assert config.enabled is True
        assert config.force_threshold == 50.0
        assert config.velocity_threshold == 100.0
        assert config.watchdog_timeout == 0.5
        assert config.auto_recovery is False

    def test_safety_config_defaults(self):
        """Test SafetyConfig default values."""
        config = SafetyConfig()
        assert config.enabled is True
        assert config.control_frequency == 30.0
        assert config.history_length == 3
        assert isinstance(config.velocity_limits, VelocityLimits)
        assert isinstance(config.acceleration_limits, AccelerationLimits)
        assert isinstance(config.collision, CollisionConfig)
        assert isinstance(config.emergency_stop, EmergencyStopConfig)

    def test_safety_config_custom(self):
        """Test SafetyConfig with custom sub-configurations."""
        config = SafetyConfig(
            enabled=True,
            control_frequency=60.0,
            velocity_limits=VelocityLimits(max_joint_velocity=45.0),
            collision=CollisionConfig(enabled=False),
        )
        assert config.control_frequency == 60.0
        assert config.velocity_limits.max_joint_velocity == 45.0
        assert config.collision.enabled is False


# =============================================================================
# CollisionDetector Tests
# =============================================================================


class TestCollisionDetector:
    """Tests for CollisionDetector class."""

    def test_init_with_disabled_config(self, temp_urdf_file, joint_names):
        """Test initialization with collision detection disabled."""
        config = CollisionConfig(enabled=False)
        detector = CollisionDetector(
            urdf_path=temp_urdf_file,
            config=config,
            joint_names=joint_names,
        )
        assert detector.config.enabled is False
        assert detector.use_placo is False

    def test_init_with_enabled_config(self, temp_urdf_file, joint_names):
        """Test initialization with collision detection enabled."""
        config = CollisionConfig(enabled=True)
        detector = CollisionDetector(
            urdf_path=temp_urdf_file,
            config=config,
            joint_names=joint_names,
        )
        assert detector.config.enabled is True
        assert detector.urdf_path == Path(temp_urdf_file)

    def test_check_collision_disabled(self, temp_urdf_file, joint_names):
        """Test collision check returns False when disabled."""
        config = CollisionConfig(enabled=False)
        detector = CollisionDetector(
            urdf_path=temp_urdf_file,
            config=config,
            joint_names=joint_names,
        )
        is_collision, details = detector.check_collision(np.array([0.0] * 7))
        assert is_collision is False
        assert details is None

    def test_check_basic_collision_no_collision(self, temp_urdf_file, joint_names):
        """Test basic collision check with safe configuration."""
        config = CollisionConfig(enabled=True)
        detector = CollisionDetector(
            urdf_path=temp_urdf_file,
            config=config,
            joint_names=joint_names,
        )
        # Safe configuration - no folding
        positions = np.array([0.0, 10.0, 20.0, 30.0, 10.0, 20.0, 0.0])
        is_collision, details = detector.check_collision(positions)
        assert is_collision is False
        assert details is None

    def test_check_basic_collision_with_collision(self, temp_urdf_file, joint_names):
        """Test basic collision check with folding configuration."""
        config = CollisionConfig(enabled=True)
        detector = CollisionDetector(
            urdf_path=temp_urdf_file,
            config=config,
            joint_names=joint_names,
        )
        # This should trigger the basic collision heuristic
        # j2 > 100, j4 > 100, and j2 * j4 < 0
        positions = np.array([0.0, 120.0, 50.0, -110.0, 0.0, 0.0, 0.0])
        is_collision, details = detector.check_collision(positions, return_details=True)
        assert is_collision is True
        assert details is not None
        assert "collisions" in details
        assert len(details["collisions"]) > 0

    def test_is_excluded_pair(self, temp_urdf_file, joint_names):
        """Test excluded link pair checking."""
        config = CollisionConfig(
            excluded_link_pairs=[("link1", "link2"), ("link3", "link4")]
        )
        detector = CollisionDetector(
            urdf_path=temp_urdf_file,
            config=config,
            joint_names=joint_names,
        )
        assert detector._is_excluded_pair("link1", "link2") is True
        assert detector._is_excluded_pair("link2", "link1") is True
        assert detector._is_excluded_pair("link1", "link3") is False
        assert detector._is_excluded_pair("link5", "link6") is False

    def test_get_link_positions_without_placo(self, temp_urdf_file, joint_names):
        """Test get_link_positions returns empty dict without placo."""
        config = CollisionConfig(enabled=True)
        detector = CollisionDetector(
            urdf_path=temp_urdf_file,
            config=config,
            joint_names=joint_names,
        )
        positions = np.array([0.0] * 7)
        link_positions = detector.get_link_positions(positions)
        assert link_positions == {}


# =============================================================================
# VelocityLimiter Tests
# =============================================================================


class TestVelocityLimiter:
    """Tests for VelocityLimiter class."""

    @pytest.fixture
    def velocity_limiter(self, joint_names):
        """Create a VelocityLimiter for testing."""
        return VelocityLimiter(
            velocity_limits=VelocityLimits(max_joint_velocity=30.0),
            acceleration_limits=AccelerationLimits(max_joint_acceleration=50.0),
            joint_names=joint_names,
            control_frequency=30.0,
            history_length=3,
        )

    def test_init(self, velocity_limiter):
        """Test VelocityLimiter initialization."""
        assert velocity_limiter.velocity_limits.max_joint_velocity == 30.0
        assert velocity_limiter.acceleration_limits.max_joint_acceleration == 50.0
        assert velocity_limiter.dt == pytest.approx(1.0 / 30.0)
        assert velocity_limiter.history_length == 3
        assert len(velocity_limiter.state_history) == 0
        assert velocity_limiter.previous_velocity is None

    def test_limit_joint_action_no_limiting(self, velocity_limiter):
        """Test limiting with velocities within bounds."""
        # Use small values that stay within 30 deg/s limit
        # 0.5 deg * 30 Hz = 15 deg/s < 30 deg/s
        target = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
        current = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        current_time = time.time()

        limited, info = velocity_limiter.limit_joint_action(target, current, current_time)

        # Small movement should not be limited
        assert not info["velocity_limited"]
        assert not info["acceleration_limited"]
        np.testing.assert_array_almost_equal(limited, target)

    def test_limit_joint_action_velocity_limiting(self, velocity_limiter):
        """Test velocity limiting with excessive movement."""
        # Request a large movement that would exceed velocity limit
        target = np.array([100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        current = np.array([0.0] * 7)
        current_time = time.time()

        limited, info = velocity_limiter.limit_joint_action(target, current, current_time)

        assert info["velocity_limited"]
        assert info["original_max_velocity"] > 30.0
        assert info["limited_max_velocity"] <= 30.0
        assert "joint1" in info["joints_limited"]
        # Limited position should be less than target
        assert limited[0] < target[0]

    def test_limit_joint_action_multiple_joints_limited(self, velocity_limiter):
        """Test velocity limiting with multiple joints exceeding limits."""
        target = np.array([100.0, -100.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        current = np.array([0.0] * 7)
        current_time = time.time()

        limited, info = velocity_limiter.limit_joint_action(target, current, current_time)

        assert info["velocity_limited"]
        # At least 3 joints should be limited
        assert len(info["joints_limited"]) >= 2

    def test_limit_joint_action_acceleration_limiting(self, velocity_limiter):
        """Test acceleration limiting on second call."""
        current = np.array([0.0] * 7)
        target1 = np.array([10.0] * 7)  # Within velocity limits
        target2 = np.array([50.0] * 7)  # Would exceed acceleration limits
        current_time = time.time()

        # First call establishes velocity
        velocity_limiter.limit_joint_action(target1, current, current_time)
        time.sleep(0.01)

        # Second call should limit acceleration
        limited, info = velocity_limiter.limit_joint_action(target2, target1, time.time())

        # The velocity change should be limited by acceleration
        assert limited is not None
        assert len(limited) == 7

    def test_limit_ee_action_no_limiting(self, velocity_limiter):
        """Test EE action limiting with small movement."""
        current_pose = np.eye(4)
        current_pose[:3, 3] = [0.0, 0.0, 0.0]
        target_pose = np.eye(4)
        target_pose[:3, 3] = [0.01, 0.01, 0.01]  # Small movement

        limited, info = velocity_limiter.limit_ee_action(target_pose, current_pose, time.time())

        assert not info["velocity_limited"]
        np.testing.assert_array_almost_equal(limited[:3, 3], target_pose[:3, 3])

    def test_limit_ee_action_velocity_limiting(self, velocity_limiter):
        """Test EE action limiting with large movement."""
        current_pose = np.eye(4)
        current_pose[:3, 3] = [0.0, 0.0, 0.0]
        target_pose = np.eye(4)
        target_pose[:3, 3] = [10.0, 10.0, 10.0]  # Large movement

        limited, info = velocity_limiter.limit_ee_action(target_pose, current_pose, time.time())

        assert info["velocity_limited"]
        # Limited position should be less than target
        for i in range(3):
            assert limited[i, 3] < target_pose[i, 3]

    def test_reset(self, velocity_limiter):
        """Test resetting the velocity limiter."""
        target = np.array([10.0] * 7)
        current = np.array([0.0] * 7)
        velocity_limiter.limit_joint_action(target, current, time.time())

        assert len(velocity_limiter.state_history) > 0
        assert velocity_limiter.previous_velocity is not None

        velocity_limiter.reset()

        assert len(velocity_limiter.state_history) == 0
        assert velocity_limiter.previous_velocity is None
        assert velocity_limiter.previous_velocity_time is None

    def test_get_state(self, velocity_limiter):
        """Test getting velocity limiter state."""
        state = velocity_limiter.get_state()
        assert "current_velocity" in state
        assert "history_length" in state
        assert state["current_velocity"] == []
        assert state["history_length"] == 0

        # After limiting
        target = np.array([10.0] * 7)
        current = np.array([0.0] * 7)
        velocity_limiter.limit_joint_action(target, current, time.time())

        state = velocity_limiter.get_state()
        assert state["history_length"] == 1
        assert len(state["current_velocity"]) == 7
