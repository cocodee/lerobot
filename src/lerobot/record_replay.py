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
Replays the actions of an episode from a dataset on a robot.

Examples:

```shell
python -m lerobot.replay \
    --robot.type=so100_follower \
    --robot.port=/dev/tty.usbmodem58760431541 \
    --robot.id=black \
    --dataset.repo_id=aliberts/record-test \
    --dataset.episode=2
```

Example replay with bimanual so100:
```shell
python -m lerobot.replay \
  --robot.type=bi_so100_follower \
  --robot.left_arm_port=/dev/tty.usbmodem5A460851411 \
  --robot.right_arm_port=/dev/tty.usbmodem5A460812391 \
  --robot.id=bimanual_follower \
  --dataset.repo_id=${HF_USER}/bimanual-so100-handover-cube \
  --dataset.episode=0
```

```shell record
python -m lerobot.record_replay \
    --robot.type=supre_robot_follower \
    --robot.id=supre_robot_follower \
    --robot.cameras="{ head_cam: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30}, right_wrist_cam: {type: opencv, index_or_path: 4, width: 640, height: 480, fps: 30}, left_wrist_cam: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}" \
    --dataset.repo_id=supdata/dataset_1128_3 \
    --dataset.episode=2\
    --record_dataset.single_task="Grasp the workpiece and put it in the appropriate position." \
    --record_dataset.repo_id=supdata/dataset_1127_5 \
    --record_dataset.episode_time_s=60 \
    --record_dataset.num_episodes=50 \
    --record_dataset.reset_time_s=5 \
    --record_dataset.push_to_hub=False \
    --record_dataset.fps=30 \
    --config_path=src/lerobot/robots/supre_robot_follower/trunk_config.yaml
```

"""

import logging
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from pprint import pformat

import draccus

from lerobot.cameras import (  # noqa: F401
    CameraConfig,  # noqa: F401
)
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig  # noqa: F401

from lerobot.configs import parser
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.robots import (  # noqa: F401
    Robot,
    RobotConfig,
    bi_so100_follower,
    hope_jr,
    koch_follower,
    make_robot_from_config,
    so100_follower,
    so101_follower,
    ros2_follower,
    supre_robot_follower
)
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.utils import (
    init_logging,
    log_say,
)
# from lerobot.processor import make_default_processors
from lerobot.datasets.utils import build_dataset_frame, hw_to_dataset_features
from lerobot.datasets.image_writer import safe_stop_image_writer
from lerobot.utils.control_utils import init_keyboard_listener


@dataclass
class DatasetRecordConfig:
    # Dataset identifier. By convention it should match '{hf_username}/{dataset_name}' (e.g. `lerobot/test`).
    repo_id: str
    # A short but accurate description of the task performed during the recording (e.g. "Pick the Lego block and drop it in the box on the right.")
    single_task: str
    # Root directory where the dataset will be stored (e.g. 'dataset/path').
    root: str | Path | None = None
    # Limit the frames per second.
    fps: int = 30
    # Number of seconds for data recording for each episode.
    episode_time_s: int | float = 60
    # Number of seconds for resetting the environment after each episode.
    reset_time_s: int | float = 60
    # Number of episodes to record.
    num_episodes: int = 50
    # Encode frames in the dataset into video
    video: bool = True
    # Upload dataset to Hugging Face hub.
    push_to_hub: bool = True
    # Upload on private repository on the Hugging Face hub.
    private: bool = False
    # Add tags to your dataset on the hub.
    tags: list[str] | None = None
    # Number of subprocesses handling the saving of frames as PNG. Set to 0 to use threads only;
    # set to ≥1 to use subprocesses, each using threads to write images. The best number of processes
    # and threads depends on your system. We recommend 4 threads per camera with 0 processes.
    # If fps is unstable, adjust the thread count. If still unstable, try using 1 or more subprocesses.
    num_image_writer_processes: int = 0
    # Number of threads writing the frames as png images on disk, per camera.
    # Too many threads might cause unstable teleoperation fps due to main thread being blocked.
    # Not enough threads might cause low camera fps.
    num_image_writer_threads_per_camera: int = 4

    def __post_init__(self):
        if self.single_task is None:
            raise ValueError("You need to provide a task as argument in `single_task`.")


@dataclass
class DatasetReplayConfig:
    # Dataset identifier. By convention it should match '{hf_username}/{dataset_name}' (e.g. `lerobot/test`).
    repo_id: str
    # Episode to replay.
    episode: int
    # Root directory where the dataset will be stored (e.g. 'dataset/path').
    root: str | Path | None = None
    # Limit the frames per second. By default, uses the policy fps.
    fps: int = 30


@dataclass
class ReplayConfig:
    robot: RobotConfig
    dataset: DatasetReplayConfig
    record_dataset: DatasetRecordConfig
    # Use vocal synthesis to read events.
    play_sounds: bool = True


@safe_stop_image_writer
def record_loop(
    robot: Robot,
    action: dict,
    dataset: LeRobotDataset | None = None,
    single_task: str | None = None,
    # robot_observation_processor: RobotProcessorPipeline[
    #     RobotObservation, RobotObservation
    # ],  # runs after robot
    ):

    # Get robot observation
    observation = robot.get_observation()
    # Applies a pipeline to the raw robot observation, default is IdentityProcessor
    # obs_processed = robot_observation_processor(obs)

    if dataset is not None:
        observation_frame = build_dataset_frame(dataset.features, observation, prefix="observation")


    final_action_dict = robot.send_action(action)
    
    # Write to dataset
    if dataset is not None:
        action_frame = build_dataset_frame(dataset.features, final_action_dict, prefix="action")
        frame = {**observation_frame, **action_frame}
        dataset.add_frame(frame, task=single_task)

        




@draccus.wrap()
def replay(cfg: ReplayConfig):
    init_logging()
    logging.info(pformat(asdict(cfg)))
    
    robot = make_robot_from_config(cfg.robot)
    dataset = LeRobotDataset(cfg.dataset.repo_id, root=cfg.dataset.root, episodes=[cfg.dataset.episode])
    actions = dataset.hf_dataset.select_columns("action")
    
    # 准备recore dataset
    action_features = hw_to_dataset_features(robot.action_features, "action", cfg.record_dataset.video)
    obs_features = hw_to_dataset_features(robot.observation_features, "observation", cfg.record_dataset.video)
    dataset_features = {**action_features, **obs_features}

    record_dataset = LeRobotDataset.create(
            cfg.record_dataset.repo_id,
            cfg.record_dataset.fps,
            root=cfg.record_dataset.root,
            robot_type=robot.name,
            features=dataset_features,
            use_videos=cfg.record_dataset.video,
            image_writer_processes=cfg.record_dataset.num_image_writer_processes,
            image_writer_threads=cfg.record_dataset.num_image_writer_threads_per_camera * len(robot.cameras),
        )
    
    
    robot.connect()
    
    listener, events = init_keyboard_listener()

    log_say("Replaying episode", cfg.play_sounds, blocking=True)
    recorded_episodes = 0
    while True:
        start_time = time.perf_counter()
        for idx in range(dataset.num_frames):
            start_episode_t = time.perf_counter()

            action_array = actions[idx]["action"]
            action = {}
            for i, name in enumerate(dataset.features["action"]["names"]):
                action[name] = action_array[i]

            if events["exit_early"]:
                events["exit_early"] = False
                break
            
            # robot.send_action(action)
            record_loop(
                robot=robot,
                action=action,
                dataset=record_dataset,
                single_task=cfg.record_dataset.single_task,
            )

            dt_s = time.perf_counter() - start_episode_t
            busy_wait(1 / dataset.fps - dt_s)
            
            dur_time = time.perf_counter() - start_time
            print("ssfdfsfsf:", dur_time)
            if dur_time > 185:
                break
        
        record_dataset.save_episode()
        recorded_episodes += 1
        if recorded_episodes > cfg.record_dataset.num_episodes - 1:
            break
    
    robot.disconnect()

    if listener is not None:
        listener.stop()


if __name__ == "__main__":
    replay()
