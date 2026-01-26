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
Teleoperation script for controlling SimRobotPandaHil using WebXR teleoperator.

Example:

```shell
# Load configuration from file
python -m lerobot.scripts.teleoperate_webxr_panda --config_path=src/lerobot/configs/teleoperate_webxr_panda.yaml

# Override config from command line
python -m lerobot.scripts.teleoperate_webxr_panda --config_path=src/lerobot/configs/teleoperate_webxr_panda.yaml --fps=20 --teleop_time_s=60
```

Or using the run script:

```shell
bash src/lerobot/scripts/run_teleoperate_webxr_panda.sh --config_path=src/lerobot/configs/teleoperate_webxr_panda.yaml
```
"""

import logging
import time
from dataclasses import dataclass, field
from typing import Any

from lerobot.robots.sim_robot_panda.sim_robot_panda_hil import SimRobotPandaHil
from lerobot.robots.sim_robot.sim_robot_hil import SimRobotHil
from lerobot.robots.sim_robot.config_sim_robot import SimRobotPandaHilConfig, SimRobotConfig
from lerobot.teleoperators.webxr.teleop_webxr import WebxrTeleop
from lerobot.teleoperators.webxr.configuration_webxr import WebxrTeleopConfig
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.utils import init_logging
from lerobot.configs import parser


@dataclass
class TeleoperateWebxrPandaConfig:
    """Configuration for WebXR teleoperation of SimRobotPandaHil."""

    robot: Any = field(default_factory=SimRobotPandaHilConfig)
    robot_type: str = "sim_robot_panda_hil"
    teleop: WebxrTeleopConfig = field(default_factory=WebxrTeleopConfig)
    fps: int = 30
    teleop_time_s: float | None = None


def teleop_loop(
    teleop: WebxrTeleop,
    robot,
    fps: int,
    duration: float | None = None,
):
    """Main teleoperation loop."""
    start = time.perf_counter()
    loop_count = 0

    logging.info("Starting teleoperation loop...")
    logging.info(f"Target FPS: {fps}")
    logging.info(f"Duration: {duration}s" if duration else "Duration: unlimited")

    while True:
        loop_start = time.perf_counter()

        # Get action from WebXR teleoperator
        action = teleop.get_action()

        # Send action to robot
        robot.send_action(action)

        # Control loop timing
        dt_s = time.perf_counter() - loop_start
        print(f"loop time: {dt_s * 1e3:.3f}ms ({1 / dt_s:.0f} Hz) | action: {action}")

        busy_wait(max(0, 1 / fps - dt_s))
        loop_s = time.perf_counter() - loop_start

        loop_count += 1

        # Check duration limit
        if duration is not None and time.perf_counter() - start >= duration:
            logging.info(f"Reached duration limit of {duration}s, stopping...")
            break

    logging.info(f"Teleoperation completed after {loop_count} loops")


@parser.wrap()
def teleoperate(cfg: TeleoperateWebxrPandaConfig):
    """Main function to set up and run teleoperation."""
    init_logging()
    logging.info("Configuration:")
    logging.info(f"  Robot: {cfg.robot}")
    logging.info(f"  Teleop: {cfg.teleop}")
    logging.info(f"  FPS: {cfg.fps}")

    # Create teleoperator and robot instances
    teleop = WebxrTeleop(cfg.teleop)

    if cfg.robot_type == "sim_robot_panda_hil":
        robot = SimRobotPandaHil(cfg.robot)
    elif cfg.robot_type == "sim_robot_hil":
        robot = SimRobotHil(cfg.robot)
    else:
        raise ValueError(f"Unknown robot type: {cfg.robot_type}")

    # Connect to teleoperator and robot
    logging.info("Connecting to WebXR teleoperator...")
    teleop.connect()
    logging.info("WebXR teleoperator connected.")

    logging.info(f"Connecting to {cfg.robot_type.capitalize()}...")
    robot.connect()
    logging.info(f"{cfg.robot_type.capitalize()} connected.")

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
        logging.info("Disconnecting...")
        try:
            teleop.disconnect()
            logging.info("WebXR teleoperator disconnected.")
        except Exception as e:
            logging.warning(f"Error disconnecting teleoperator: {e}")

        try:
            robot.disconnect()
            logging.info(f"{cfg.robot_type.capitalize()} disconnected.")
        except Exception as e:
            logging.warning(f"Error disconnecting robot: {e}")

    logging.info("Teleoperation session ended.")


if __name__ == "__main__":
    teleoperate()
