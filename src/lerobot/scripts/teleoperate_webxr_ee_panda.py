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
Teleoperation script for controlling SimRobotPandaHil using WebXR Delta teleoperator.

This version uses WebxrDeltaTeleop which outputs delta actions (like other teleop devices),
making it compatible with the standard RL training pipeline.

The WebxrDeltaTeleop internally uses WebXRIntentTranslator to convert WebXR poses
into delta actions based on the current mode (TRANSLATE, ROTATE, BOTH, IDLE).

Example:

```shell
# Load configuration from file
python -m lerobot.scripts.teleoperate_webxr_ee_panda --config_path=src/lerobot/configs/teleoperate_webxr_panda.yaml

# Override config from command line
python -m lerobot.scripts.teleoperate_webxr_ee_panda --config_path=src/lerobot/configs/teleoperate_webxr_panda.yaml --fps=20 --teleop_time_s=60
```

Key differences from teleoperate_webxr_panda.py:
- Uses WebxrDeltaTeleop instead of WebxrTeleop
- Outputs delta actions instead of absolute poses
- Compatible with HIL-SERL training pipeline
"""

import logging
import time
from dataclasses import dataclass, field
from typing import Any

from lerobot.robots.sim_robot_panda.sim_robot_panda_hil import SimRobotPandaHil
from lerobot.robots.sim_robot.sim_robot_hil import SimRobotHil
from lerobot.robots.sim_robot.config_sim_robot import SimRobotPandaHilConfig, SimRobotConfig
from lerobot.teleoperators.webxr.teleop_webxr_delta import WebxrDeltaTeleop
from lerobot.teleoperators.webxr.configuration_webxr import WebxrTeleopConfig
from lerobot.robots import (  # noqa: F401
    Robot,
    RobotConfig,
)
from lerobot.teleoperators import (  # noqa: F401
    Teleoperator,
    TeleoperatorConfig,
)
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.utils import init_logging, log_say
from lerobot.configs import parser


@dataclass
class TeleoperateWebxrEePandaConfig:
    """Configuration for WebXR Delta teleoperation of SimRobotPandaHil."""

    robot: RobotConfig
    teleop: TeleoperatorConfig
    robot_type: str = "sim_robot_panda_hil"
    fps: int = 30
    teleop_time_s: float | None = None


def teleop_loop(
    teleop: WebxrDeltaTeleop,
    robot,
    fps: int,
    duration: float | None = None,
):
    """
    Main teleoperation loop with WebXR Delta teleop.

    Key difference: WebxrDeltaTeleop outputs delta actions that are
    compatible with the robot's send_action() interface.

    Args:
        teleop: WebxrDeltaTeleop instance
        robot: Robot instance (e.g., SimRobotPandaHil)
        fps: Target frames per second
        duration: Optional duration in seconds
    """
    start = time.perf_counter()
    loop_count = 0

    logging.info("Starting WebXR Delta teleoperation loop...")
    logging.info(f"Target FPS: {fps}")
    logging.info(f"Duration: {duration}s" if duration else "Duration: unlimited")
    logging.info("")
    logging.info("WebXR Modes:")
    logging.info("  IDLE      - No action, robot stays still")
    logging.info("  TRANSLATE - Move in X/Y/Z direction")
    logging.info("  ROTATE    - Rotate orientation")
    logging.info("  BOTH      - Translate + Rotate simultaneously")
    logging.info("")
    logging.info("Action format: {delta_x, delta_y, delta_z, delta_qx, delta_qy, delta_qz, delta_qw, gripper}")

    while True:
        loop_start = time.perf_counter()

        # Get delta action from WebXR Delta teleoperator
        # Note: WebxrDeltaTeleop internally fetches current robot pose
        # and uses WebXRIntentTranslator to compute delta
        action = teleop.get_action()

        # Log action details
        mode = action.get("mode", "IDLE")
        if mode != "IDLE":
            logging.info(
                f"Action: mode={mode}, "
                f"delta_pos=[{action['delta_x']:.4f}, {action['delta_y']:.4f}, {action['delta_z']:.4f}], "
                f"gripper={action['gripper']:.2f}"
            )

        # Send action to robot (robot expects delta format)
        robot.send_action(action)

        # Control loop timing
        dt_s = time.perf_counter() - loop_start

        if loop_count % 30 == 0:  # Log every 30 loops
            logging.info(f"Loop {loop_count}: {dt_s * 1e3:.3f}ms ({1 / dt_s:.0f} Hz)")

        busy_wait(max(0, 1 / fps - dt_s))

        loop_count += 1

        # Check duration limit
        if duration is not None and time.perf_counter() - start >= duration:
            logging.info(f"Reached duration limit of {duration}s, stopping...")
            break


@parser.wrap()
def teleoperate(cfg: TeleoperateWebxrEePandaConfig):
    """
    Main function to set up and run WebXR Delta teleoperation.

    Args:
        cfg: Configuration containing robot and teleop settings
    """
    init_logging()
    logging.info("=" * 60)
    logging.info("WebXR Delta Teleoperation")
    logging.info("=" * 60)
    logging.info("Configuration:")
    logging.info(f"  Robot: {cfg.robot}")
    logging.info(f"  Teleop: {cfg.teleop}")
    logging.info(f"  FPS: {cfg.fps}")
    logging.info(f"  Robot Type: {cfg.robot_type}")

    # Create teleoperator and robot instances
    logging.info("")
    logging.info("Creating WebxrDeltaTeleop...")
    teleop = WebxrDeltaTeleop(cfg.teleop)

    logging.info(f"Creating {cfg.robot_type}...")
    if cfg.robot_type == "sim_robot_panda_hil":
        robot = SimRobotPandaHil(cfg.robot)
    elif cfg.robot_type == "sim_robot_hil":
        robot = SimRobotHil(cfg.robot)
    else:
        raise ValueError(f"Unknown robot type: {cfg.robot_type}")

    # Connect to teleoperator and robot
    logging.info("")
    logging.info("Connecting to WebXR Delta teleoperator...")
    teleop.connect()

    logging.info(f"Connecting to {cfg.robot_type}...")
    robot.connect()

    # Set robot reference for automatic pose updates
    # This allows WebxrDeltaTeleop to fetch current EE pose automatically
    teleop.set_robot(robot)
    logging.info("Robot reference set for automatic pose updates")

    logging.info("")
    log_say("WebXR Delta teleoperation ready!", play_sounds=True)
    logging.info("")
    logging.info("Controls (via WebXR interface):")
    logging.info("  Use VR controller to switch between modes:")
    logging.info("    - IDLE: No movement")
    logging.info("    - TRANSLATE: Move position")
    logging.info("    - ROTATE: Rotate orientation")
    logging.info("    - BOTH: Position + Rotation")
    logging.info("")

    try:
        # Run teleoperation loop
        teleop_loop(
            teleop=teleop,
            robot=robot,
            fps=cfg.fps,
            duration=cfg.teleop_time_s,
        )
    except KeyboardInterrupt:
        logging.info("KeyboardInterrupt received, stopping teleoperation...")
    except Exception as e:
        logging.error(f"Error during teleoperation: {e}", exc_info=True)
        raise
    finally:
        # Clean up
        logging.info("")
        logging.info("Disconnecting...")
        try:
            teleop.disconnect()
            logging.info("WebXR Delta teleoperator disconnected.")
        except Exception as e:
            logging.warning(f"Error disconnecting teleoperator: {e}")

        try:
            robot.disconnect()
            logging.info(f"{cfg.robot_type} disconnected.")
        except Exception as e:
            logging.warning(f"Error disconnecting robot: {e}")

    logging.info("Teleoperation session ended.")


if __name__ == "__main__":
    teleoperate()
