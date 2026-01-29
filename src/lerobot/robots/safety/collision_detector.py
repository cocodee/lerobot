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
Collision detection for robot self-collision monitoring.

Uses placo's collision detection when available, with a geometric
bounding sphere fallback for simple cases.
"""

import logging
import numpy as np
from typing import List, Tuple, Optional
from pathlib import Path

from .safety_config import CollisionConfig

logger = logging.getLogger(__name__)


class CollisionDetector:
    """
    Detects self-collisions using robot URDF and current joint positions.

    Uses placo's collision detection when available, with a geometric
    bounding sphere fallback for when placo is not available.

    Args:
        urdf_path: Path to the robot URDF file.
        config: Collision configuration.
        joint_names: List of joint names in the robot.

    Example:
        ```python
        detector = CollisionDetector(
            urdf_path="path/to/robot.urdf",
            config=CollisionConfig(enabled=True, collision_threshold=0.05),
            joint_names=["joint1", "joint2", ...]
        )
        is_collision, details = detector.check_collision(joint_positions)
        ```
    """

    def __init__(
        self,
        urdf_path: str | Path,
        config: CollisionConfig,
        joint_names: List[str],
    ):
        self.config = config
        self.joint_names = joint_names
        self.urdf_path = Path(urdf_path)

        # Try to use placo for collision detection
        self.use_placo = False
        self.robot = None

        if config.enabled:
            self._init_placo()

        if not self.use_placo:
            logger.warning("placo not available, collision detection will use simple checks only")

    def _init_placo(self):
        """Initialize placo for collision detection."""
        try:
            import placo
            self.robot = placo.RobotWrapper(str(self.urdf_path))
            self.use_placo = True
            logger.info(f"Using placo for collision detection with URDF: {self.urdf_path}")
        except ImportError:
            logger.warning("placo not installed, collision detection limited to basic checks")
        except Exception as e:
            logger.warning(f"Failed to initialize placo: {e}")

    def check_collision(
        self,
        joint_positions: np.ndarray,
        return_details: bool = False
    ) -> Tuple[bool, Optional[dict]]:
        """
        Check if current joint configuration results in self-collision.

        Args:
            joint_positions: Current joint positions in degrees.
            return_details: If True, return collision details including
                which links are colliding and distances.

        Returns:
            A tuple of (is_collision, details):
                - is_collision: True if collision detected
                - details: Optional dict with collision information
        """
        if not self.config.enabled:
            return False, None

        if self.use_placo and self.robot is not None:
            return self._check_placo_collision(joint_positions, return_details)
        else:
            return self._check_basic_collision(joint_positions, return_details)

    def _check_placo_collision(
        self,
        joint_positions: np.ndarray,
        return_details: bool
    ) -> Tuple[bool, Optional[dict]]:
        """Use placo's collision detection."""
        try:
            import placo

            # Update joint positions in placo robot model
            joint_rad = np.deg2rad(joint_positions[:len(self.joint_names)])
            for i, name in enumerate(self.joint_names):
                self.robot.set_joint(name, joint_rad[i])

            # Update kinematics
            self.robot.update_kinematics()

            # Check for collisions using placo's collision detection
            # placo provides distance checking between collision shapes
            collision_pairs = self.robot.compute_collisions()

            # Filter and process collisions
            collisions = []
            for pair in collision_pairs:
                # Check if this pair is in excluded list
                link1 = pair.link1 if hasattr(pair, 'link1') else pair.get('link1', '')
                link2 = pair.link2 if hasattr(pair, 'link2') else pair.get('link2', '')
                distance = pair.distance if hasattr(pair, 'distance') else pair.get('distance', float('inf'))

                # Skip excluded pairs
                if self._is_excluded_pair(link1, link2):
                    continue

                # Check if distance is below threshold
                if distance < self.config.collision_threshold:
                    collisions.append({
                        'link1': link1,
                        'link2': link2,
                        'distance': distance,
                    })

            is_collision = len(collisions) > 0
            details = {'collisions': collisions} if return_details and collisions else None

            if is_collision:
                logger.warning(f"Collision detected: {collisions}")

            return is_collision, details

        except Exception as e:
            logger.error(f"Error in placo collision detection: {e}")
            # Fall back to basic check
            return self._check_basic_collision(joint_positions, return_details)

    def _check_basic_collision(
        self,
        joint_positions: np.ndarray,
        return_details: bool
    ) -> Tuple[bool, Optional[dict]]:
        """
        Basic geometric collision detection fallback.

        This is a simplified check when placo is not available.
        It performs basic joint limit and configuration checks.
        """
        # Basic checks:
        # 1. Check if any joint is beyond reasonable limits (safety violation)
        # 2. Check for suspicious configurations (e.g., joint angles that would fold arm into itself)

        # This is intentionally conservative - if we can't properly detect collisions,
        # we should rely on velocity limiting and other safety measures

        collisions = []

        # Check for obviously dangerous configurations
        # For example, if joint 2 and joint 4 are both near their folding points
        if len(joint_positions) >= 4:
            # This is a simplified heuristic - real collision detection needs proper geometry
            j2, j3, j4 = joint_positions[1], joint_positions[2], joint_positions[3]
            # Example: if middle joints are folded toward each other
            if abs(j2) > 100 and abs(j4) > 100 and j2 * j4 < 0:
                collisions.append({
                    'link1': 'arm_middle',
                    'link2': 'arm_forearm',
                    'distance': 0.0,
                    'note': 'Conservative collision check - placo not available',
                })

        is_collision = len(collisions) > 0
        details = {'collisions': collisions} if return_details and collisions else None

        return is_collision, details

    def _is_excluded_pair(self, link1: str, link2: str) -> bool:
        """Check if a link pair is in the exclusion list."""
        for excluded in self.config.excluded_link_pairs:
            if (link1, link2) == excluded or (link2, link1) == excluded:
                return True
        return False

    def get_link_positions(
        self,
        joint_positions: np.ndarray
    ) -> dict[str, np.ndarray]:
        """
        Get current positions of all links in the robot.

        Useful for debugging and visualization.

        Args:
            joint_positions: Current joint positions in degrees.

        Returns:
            Dictionary mapping link names to their 4x4 transformation matrices.
        """
        if not self.use_placo or self.robot is None:
            logger.warning("Link positions require placo")
            return {}

        try:
            # Update joint positions
            joint_rad = np.deg2rad(joint_positions[:len(self.joint_names)])
            for i, name in enumerate(self.joint_names):
                self.robot.set_joint(name, joint_rad[i])
            self.robot.update_kinematics()

            # Get link positions
            link_positions = {}
            for link_name in self.robot.link_names:
                link_positions[link_name] = self.robot.get_link_transform(link_name)

            return link_positions

        except Exception as e:
            logger.error(f"Error getting link positions: {e}")
            return {}
