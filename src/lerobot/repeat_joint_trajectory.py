#!/usr/bin/env python

"""
Repeat a fixed sequence of joint targets for TCP repeatability measurements.

This script only moves the robot through repeatable joint targets. Measure the
end-effector TCP repeatability with an external instrument at each settle point.

Example:

python -m lerobot.repeat_joint_trajectory \
    --robot.type=supre_robot_follower \
    --cycles=10 \
    --trajectory_duration=6.0 \
    --settle_time_s=2.0 \
    --pause_for_measurement=true
"""

import logging
import time
from dataclasses import asdict, dataclass, field
from pprint import pformat

import draccus

from lerobot.robots import Robot, RobotConfig, make_robot_from_config
from lerobot.robots import supre_robot_follower  # noqa: F401
from lerobot.utils.utils import init_logging


SUPRE_JOINT_NAMES = [
    "left_arm_joint_1",
    "left_arm_joint_2",
    "left_arm_joint_3",
    "left_arm_joint_4",
    "left_arm_joint_5",
    "left_arm_joint_6",
    "left_arm_joint_7",
    "right_arm_joint_1",
    "right_arm_joint_2",
    "right_arm_joint_3",
    "right_arm_joint_4",
    "right_arm_joint_5",
    "right_arm_joint_6",
    "right_arm_joint_7",
]


def _target(values: list[float]) -> dict[str, float]:
    if len(values) != len(SUPRE_JOINT_NAMES):
        raise ValueError(f"Expected {len(SUPRE_JOINT_NAMES)} joint values, got {len(values)}.")
    return {f"{joint}.pos": value for joint, value in zip(SUPRE_JOINT_NAMES, values)}


def default_target_sequence() -> list[dict[str, float]]:
    """Conservative absolute joint targets for the SupreRobot follower."""
    return [
        _target(
            [
                -5.0,
                -5.0,
                0.0,
                0.0,
                5.0,
                0.0,
                0.0,
                5.0,
                10.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ]
        ),
    ]


@dataclass
class RepeatJointTrajectoryConfig:
    robot: RobotConfig
    trajectory_duration: float = 6.0
    settle_time_s: float = 2.0
    cycles: int = 10
    return_home: bool = True
    pause_for_measurement: bool = False
    startup_wait_s: float = 3.0
    target_sequence: list[dict[str, float]] = field(default_factory=default_target_sequence)


def _get_home_action(robot: Robot) -> dict[str, float]:
    if hasattr(robot, "get_current_position"):
        current_positions = robot.get_current_position()
        return {f"{joint_name}.pos": float(value) for joint_name, value in current_positions.items()}

    observation = robot.get_observation()
    return {key: float(observation[key]) for key in robot.action_features}


def _validate_config(cfg: RepeatJointTrajectoryConfig) -> None:
    if cfg.cycles < 1:
        raise ValueError(f"cycles must be >= 1, got {cfg.cycles}.")
    if cfg.trajectory_duration < 0:
        raise ValueError(f"trajectory_duration must be >= 0, got {cfg.trajectory_duration}.")
    if cfg.settle_time_s < 0:
        raise ValueError(f"settle_time_s must be >= 0, got {cfg.settle_time_s}.")
    if cfg.startup_wait_s < 0:
        raise ValueError(f"startup_wait_s must be >= 0, got {cfg.startup_wait_s}.")
    if not cfg.target_sequence:
        raise ValueError("target_sequence must contain at least one target.")


def _validate_targets(robot: Robot, targets: list[dict[str, float]]) -> None:
    required_keys = set(robot.action_features)

    for index, target in enumerate(targets, start=1):
        target_keys = set(target)
        missing_keys = sorted(required_keys - target_keys)
        extra_keys = sorted(target_keys - required_keys)

        if missing_keys:
            raise ValueError(f"Target {index} is missing required action keys: {missing_keys}")
        if extra_keys:
            raise ValueError(f"Target {index} contains unknown action keys: {extra_keys}")


def _format_target(target: dict[str, float]) -> str:
    max_key_len = max(len(key) for key in target)
    rows = [f"{key:<{max_key_len}}  {value:>9.4f}" for key, value in sorted(target.items())]
    return "\n".join(rows)


def _wait_for_measurement(pause_for_measurement: bool) -> None:
    if pause_for_measurement:
        input("Measurement point reached. Press Enter to continue...")


def execute_repeated_joint_trajectory(robot: Robot, cfg: RepeatJointTrajectoryConfig) -> None:
    home_action = _get_home_action(robot)
    _validate_targets(robot, cfg.target_sequence)

    if cfg.return_home:
        logging.info("Moving to startup home position before the repeatability sequence.")
        robot.execute_trajectory(goal_action=home_action, duration=cfg.trajectory_duration)
        time.sleep(cfg.settle_time_s)

    total_targets = len(cfg.target_sequence)
    logging.info(
        "Starting repeatability motion sequence: %s cycle(s), %s target(s) per cycle.",
        cfg.cycles,
        total_targets,
    )

    for cycle_index in range(1, cfg.cycles + 1):
        for target_index, target in enumerate(cfg.target_sequence, start=1):
            print("\n" + "=" * 72)
            print(f"Cycle {cycle_index}/{cfg.cycles} - Target {target_index}/{total_targets}")
            print(_format_target(target))
            print("=" * 72)

            start_time = time.perf_counter()
            robot.execute_trajectory(goal_action=target, duration=cfg.trajectory_duration)
            elapsed_s = time.perf_counter() - start_time

            if cfg.settle_time_s > 0:
                time.sleep(cfg.settle_time_s)

            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            print(
                f"MEASURE cycle={cycle_index} target={target_index} "
                f"time={timestamp} move_s={elapsed_s:.3f} settle_s={cfg.settle_time_s:.3f}"
            )
            _wait_for_measurement(cfg.pause_for_measurement)

    if cfg.return_home:
        logging.info("Returning to startup home position.")
        robot.execute_trajectory(goal_action=home_action, duration=cfg.trajectory_duration)
        time.sleep(cfg.settle_time_s)


@draccus.wrap()
def main(cfg: RepeatJointTrajectoryConfig) -> None:
    init_logging()
    logging.info("--- Repeat Joint Trajectory Test ---")
    logging.info(pformat(asdict(cfg)))

    _validate_config(cfg)
    robot = make_robot_from_config(cfg.robot)

    try:
        logging.info("Connecting to robot...")
        robot.connect()

        if cfg.startup_wait_s > 0:
            time.sleep(cfg.startup_wait_s)

        execute_repeated_joint_trajectory(robot, cfg)
        logging.info("Repeat joint trajectory sequence complete.")
    except KeyboardInterrupt:
        logging.info("Keyboard interrupt detected. Shutting down.")
    except Exception as exc:
        logging.error("Unexpected error during repeat joint trajectory test: %s", exc, exc_info=True)
        raise
    finally:
        logging.info("Disconnecting from robot...")
        if robot.is_connected:
            robot.disconnect()
        logging.info("Shutdown complete.")


if __name__ == "__main__":
    main()
