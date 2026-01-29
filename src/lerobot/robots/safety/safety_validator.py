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
Centralized safety validator for robot teleoperation.

Combines collision detection, velocity/acceleration limiting,
and emergency stop into a unified interface.
"""

import logging
import numpy as np
from typing import Tuple, Dict, Any, Optional, List
from pathlib import Path

from .safety_config import SafetyConfig
from .collision_detector import CollisionDetector
from .velocity_limiter import VelocityLimiter
from .emergency_stop import EmergencyStopController, EmergencyStopEvent

logger = logging.getLogger(__name__)


class SafetyValidator:
    """
    Centralized safety validator for robot teleoperation.

    Combines collision detection, velocity/acceleration limiting,
    and emergency stop into a unified interface.

    Args:
        config: Complete safety configuration.
        urdf_path: Path to robot URDF file for collision detection.
        joint_names: List of joint names in the robot.

    Example:
        ```python
        from lerobot.robots.safety import SafetyValidator, SafetyConfig

        validator = SafetyValidator(
            config=SafetyConfig(enabled=True),
            urdf_path="path/to/robot.urdf",
            joint_names=["joint1", "joint2", ...]
        )

        # In control loop
        safe_action, info = validator.validate_action(
            action=action,
            current_state=current_state,
            current_time=time.time()
        )
        ```
    """

    def __init__(
        self,
        config: SafetyConfig,
        urdf_path: str | Path,
        joint_names: List[str],
    ):
        self.config = config
        self.joint_names = joint_names
        self.urdf_path = Path(urdf_path)

        # Initialize collision detector
        self.collision_detector: Optional[CollisionDetector] = None
        if config.collision.enabled:
            try:
                self.collision_detector = CollisionDetector(
                    urdf_path=urdf_path,
                    config=config.collision,
                    joint_names=joint_names,
                )
                logger.info("Collision detector initialized")
            except Exception as e:
                logger.warning(f"Failed to initialize collision detector: {e}")

        # Initialize velocity limiter
        self.velocity_limiter = VelocityLimiter(
            velocity_limits=config.velocity_limits,
            acceleration_limits=config.acceleration_limits,
            joint_names=joint_names,
            control_frequency=config.control_frequency,
            history_length=config.history_length,
        )

        # Initialize emergency stop controller
        self.emergency_stop = EmergencyStopController(
            config=config.emergency_stop,
            on_stop_callback=self._on_emergency_stop,
        )

        # Statistics
        self.stats = {
            'actions_validated': 0,
            'collisions_prevented': 0,
            'velocity_limiting': 0,
            'acceleration_limiting': 0,
            'emergency_stops': 0,
        }

        logger.info(
            f"SafetyValidator initialized (enabled: {config.enabled}, "
            f"joints: {len(joint_names)}, URDF: {urdf_path})"
        )

    def validate_action(
        self,
        action: dict[str, Any],
        current_state: dict[str, Any],
        current_time: float,
    ) -> Tuple[dict[str, Any], dict[str, Any]]:
        """
        Validate and potentially modify an action for safety.

        This is the main entry point for safety validation. It performs
        the following checks in order:
        1. Feed watchdog (prevents timeout)
        2. Check if emergency stop is active
        3. Apply velocity/acceleration limiting
        4. Check for collisions
        5. Check emergency conditions

        Args:
            action: Desired action dict with joint positions (e.g., {"joint1.pos": 10.0, ...})
            current_state: Current robot state dict
            current_time: Current timestamp for velocity computation

        Returns:
            A tuple of (safe_action, validation_info):
                - safe_action: Validated action dict (may be modified or empty if stopped)
                - validation_info: Dict with details about what was checked/modified
        """
        validation_info = {
            'modified': False,
            'collision_detected': False,
            'velocity_limited': False,
            'acceleration_limited': False,
            'emergency_stop': False,
        }

        # Feed watchdog
        self.emergency_stop.feed_watchdog()

        # Check if stopped
        if self.emergency_stop.is_stopped():
            logger.warning("Emergency stop active, rejecting action")
            validation_info['emergency_stop'] = True
            return {}, validation_info

        # Extract joint positions from action
        action_positions = self._extract_joint_positions(action)
        current_positions = self._extract_joint_positions(current_state)

        if action_positions is None:
            logger.warning("Could not extract joint positions from action, passing through")
            return action, validation_info

        if current_positions is None:
            logger.warning("Could not extract joint positions from current state, using action as-is")
            return action, validation_info

        # Step 1: Velocity/Acceleration limiting
        limited_positions, limit_info = self.velocity_limiter.limit_joint_action(
            target_positions=action_positions,
            current_positions=current_positions,
            current_time=current_time,
        )

        if limit_info['velocity_limited']:
            validation_info['velocity_limited'] = True
            validation_info['modified'] = True
            self.stats['velocity_limiting'] += 1

        if limit_info['acceleration_limited']:
            validation_info['acceleration_limited'] = True
            validation_info['modified'] = True
            self.stats['acceleration_limiting'] += 1

        # Step 2: Collision detection on limited positions
        if self.collision_detector:
            is_collision, collision_details = self.collision_detector.check_collision(
                joint_positions=limited_positions,
                return_details=True,
            )

            if is_collision:
                validation_info['collision_detected'] = True
                self.stats['collisions_prevented'] += 1

                logger.warning(
                    f"Collision detected, triggering emergency stop. "
                    f"Details: {collision_details}"
                )

                # Trigger emergency stop
                self.emergency_stop.trigger_stop(
                    reason=f"Self-collision detected: {collision_details}",
                    source="collision",
                    current_state=current_state,
                )

                # Return current positions (don't move)
                safe_action = self._create_action_from_positions(current_positions)
                return safe_action, validation_info

        # Step 3: Check emergency conditions
        if self._check_emergency_conditions(limited_positions, current_positions):
            self.emergency_stop.trigger_stop(
                reason="Emergency velocity threshold exceeded",
                source="velocity",
                current_state=current_state,
            )
            validation_info['emergency_stop'] = True
            return {}, validation_info

        # Create safe action
        safe_action = self._create_action_from_positions(limited_positions)

        self.stats['actions_validated'] += 1
        return safe_action, validation_info

    def reset(self):
        """
        Reset safety validator state.

        Call this when starting a new episode or after a long pause.
        Resets velocity limiter state and attempts to recover from emergency stop.
        """
        self.velocity_limiter.reset()

        if self.emergency_stop.is_stopped():
            recovered = self.emergency_stop.reset(manual=True)
            if recovered:
                logger.info("Emergency stop recovered on reset")

    def get_stats(self) -> Dict[str, int]:
        """
        Get safety statistics.

        Returns:
            Dictionary with safety event counts.
        """
        return self.stats.copy()

    def is_stopped(self) -> bool:
        """Check if emergency stop is currently active."""
        return self.emergency_stop.is_stopped()

    def can_proceed(self) -> bool:
        """Check if robot can proceed with normal operation."""
        return self.emergency_stop.can_proceed()

    def manual_reset(self) -> bool:
        """
        Manually reset emergency stop.

        Returns:
            True if reset successful, False otherwise.
        """
        return self.emergency_stop.reset(manual=True)

    def _extract_joint_positions(
        self,
        state: dict[str, Any],
    ) -> Optional[np.ndarray]:
        """
        Extract joint positions from state dict.

        Handles various formats: "joint_name.pos", "joint_name", etc.

        Args:
            state: State dictionary with joint positions.

        Returns:
            Numpy array of joint positions, or None if extraction failed.
        """
        try:
            positions = []
            for name in self.joint_names:
                # Try various key formats
                if f"{name}.pos" in state:
                    positions.append(float(state[f"{name}.pos"]))
                elif name in state:
                    val = state[name]
                    # Handle if value is wrapped in array
                    if isinstance(val, (list, np.ndarray)):
                        positions.append(float(val[0] if len(val) > 0 else 0))
                    else:
                        positions.append(float(val))
                else:
                    return None
            return np.array(positions)
        except (KeyError, TypeError, ValueError, IndexError) as e:
            logger.debug(f"Failed to extract joint positions: {e}")
            return None

    def _create_action_from_positions(
        self,
        positions: np.ndarray,
    ) -> dict[str, Any]:
        """
        Create action dict from joint positions.

        Args:
            positions: Joint positions array.

        Returns:
            Action dictionary with joint_name.pos keys.
        """
        return {f"{name}.pos": float(pos) for name, pos in zip(self.joint_names, positions)}

    def _check_emergency_conditions(
        self,
        target_positions: np.ndarray,
        current_positions: np.ndarray,
    ) -> bool:
        """
        Check if emergency conditions are met.

        Currently checks for excessive velocity that could indicate
        a dangerous situation.

        Args:
            target_positions: Target joint positions.
            current_positions: Current joint positions.

        Returns:
            True if emergency conditions detected, False otherwise.
        """
        # Check for excessive velocity
        diff = np.abs(target_positions - current_positions)
        max_diff = np.max(diff)

        # Convert to velocity (rough estimate)
        max_velocity = max_diff * self.config.control_frequency

        if max_velocity > self.config.emergency_stop.velocity_threshold:
            logger.warning(
                f"Emergency velocity exceeded: {max_velocity:.1f} deg/s > "
                f"{self.config.emergency_stop.velocity_threshold:.1f} deg/s"
            )
            return True

        return False

    def _on_emergency_stop(self, event: EmergencyStopEvent):
        """
        Callback when emergency stop is triggered.

        Args:
            event: Emergency stop event with details.
        """
        self.stats['emergency_stops'] += 1
        logger.critical(
            f"Emergency stop event #{len(self.emergency_stop.stop_events)}: "
            f"{event.trigger_reason} (source: {event.trigger_source})"
        )
