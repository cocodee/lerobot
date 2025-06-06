import time
from dataclasses import replace

from lerobot.common.robot_devices.robots.configs import AgibotX1RobotConfig
from lerobot.common.robot_devices.utils import RobotDeviceAlreadyConnectedError, RobotDeviceNotConnectedError
import lerobot.common.robot_devices.motors.agibotx1 as agibotx1
from lerobot.common.robot_devices.motors.utils import MotorsBus, make_motors_buses_from_configs
from lerobot.common.robot_devices.robots.utils import get_arm_id
import json
import torch
import numpy as np
from pathlib import Path
import lerobot.common.robot_devices.robots.joy_stick_py as joy_stick_py
def ensure_safe_goal_position(
    goal_pos: torch.Tensor, present_pos: torch.Tensor, max_relative_target: float | list[float]
):
    # Cap relative action target magnitude for safety.
    diff = goal_pos - present_pos
    max_relative_target = torch.tensor(max_relative_target)
    safe_diff = torch.minimum(diff, max_relative_target)
    safe_diff = torch.maximum(safe_diff, -max_relative_target)
    safe_goal_pos = present_pos + safe_diff

    if not torch.allclose(goal_pos, safe_goal_pos):
        logging.warning(
            "Relative goal position magnitude had to be clamped to be safe.\n"
            f"  requested relative goal position target: {diff}\n"
            f"    clamped relative goal position target: {safe_diff}"
        )

    return safe_goal_pos
class AgibotX1Robot():

    def __init__(self, config: AgibotX1RobotConfig | None = None, **kwargs):
        super().__init__()
        if config is None:
            self.config = AgibotX1RobotConfig(**kwargs)
        else:
            # Overwrite config arguments using kwargs
            self.config = replace(config, **kwargs)

        self.robot_type = self.config.type
        self.is_connected = False
        self.teleop = None
        self.logs = {}
        self.calibration_dir = Path(self.config.calibration_dir)

        self.state_keys = None
        self.action_keys = None

        self.leader_arms = make_motors_buses_from_configs(self.config.leader_arms)
        self.follower_arms = make_motors_buses_from_configs(self.config.follower_arms)
        self.lumbar = make_motors_buses_from_configs(self.config.lumbar)
        self.cameras = {}
        self.if_name = self.config.if_name
        self.joy_instance = joy_stick_py.Joy()

    def connect(self) -> None:
        if self.is_connected:
            raise RobotDeviceAlreadyConnectedError(
                "AgibotX1Robot is already connected. Do not run `robot.connect()` twice."
            )

        if not self.leader_arms and not self.follower_arms and not self.cameras:
            raise ValueError(
                "AgibotX1Robot doesn't have any device to connect. See example of usage in docstring of the class."
            )
        
        created_dcu_names = []
        ## create_dcu from config
        for dcu_config in self.config.dcu_configs:
            dcu_name = dcu_config[0]
            if dcu_name in created_dcu_names:
                continue
            else:
                created_dcu_names.append(dcu_name)
                agibotx1.create_dcu(dcu_name, dcu_config[1])
        
        # Connect the arms
        for name in self.follower_arms:
            print(f"Connecting {name} follower arm.")
            self.follower_arms[name].connect()
        for name in self.leader_arms:
            print(f"Connecting {name} leader arm.")
            self.leader_arms[name].connect()
        for name in self.lumbar:
            print(f"Connecting {name} lumbar.")
            self.lumbar[name].connect()

        agibotx1.start_controller(self.if_name)
        #agibotx1.get_controller().disable_all_actuator()
        
        #TODO: torque mode?
        for name in self.follower_arms:
            self.follower_arms[name].enable_all_actuator()
        for name in self.leader_arms:
            self.leader_arms[name].disable_all_actuator()  
        for name in self.lumbar:
            self.lumbar[name].disable_all_actuator()

        #TODO: activate calibration
        self.activate_calibration()
        #TODO: set_agibotx1_robot_preset()

        # TODO:Enable torque on all motors of the follower arms?

        # Check both arms can be read
        for name in self.follower_arms:
            self.follower_arms[name].read("position")
        for name in self.leader_arms:
            self.leader_arms[name].read("position")

        #self.lumbar["main"].write("position", [0,0])
        # Connect the cameras?

        self.is_connected = True

    def run_calibration(self) -> None:
        pass

    def activate_calibration(self):
        """After calibration all motors function in human interpretable ranges.
        Rotations are expressed in degrees in nominal range of [-180, 180],
        and linear motions (like gripper of Aloha) in nominal range of [0, 100].
        """
        def load_calibration_(name, arm, arm_type):
            arm_id = get_arm_id(name, arm_type)
            arm_calib_path = self.calibration_dir / f"{arm_id}.json"

            if arm_calib_path.exists():
                with open(arm_calib_path) as f:
                    calibration = json.load(f)
            else:
                # TODO(rcadene): display a warning in __init__ if calibration file not available
                print(f"Missing calibration file '{arm_calib_path}'")

            return calibration

        for name, arm in self.follower_arms.items():
            calibration = load_calibration_(name, arm, "follower")
            arm.set_calibration(calibration)
        for name, arm in self.leader_arms.items():
            calibration = load_calibration_(name, arm, "leader")
            arm.set_calibration(calibration)

    def teleop_step(
        self, record_data=False
    ) -> None | tuple[dict[str, torch.Tensor], dict[str, torch.Tensor]]:
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "ManipulatorRobot is not connected. You need to run `robot.connect()`."
            )

        def get_arm_name(name:str,prefix:str):
            if  name.startswith(prefix):
                return name[len(prefix):]
            else:
                return name
        # Prepare to assign the position of the leader to the follower
        leader_pos = {}
        for name in self.leader_arms:
            before_lread_t = time.perf_counter()
            #name = get_arm_name(name,"left_")
            leader_pos[name] = self.leader_arms[name].read("position")
            leader_pos[name] = torch.from_numpy(leader_pos[name])
            self.logs[f"read_leader_{name}_pos_dt_s"] = time.perf_counter() - before_lread_t

        print(f'leader_pos: {leader_pos}')
        # Send goal position to the follower
        follower_goal_pos = {}
        for name in self.follower_arms:
            before_fwrite_t = time.perf_counter()
            #name = get_arm_name(name,"right_")
            goal_pos = leader_pos[name]

            claw_position = self.follower_arms[name].read("position","right_claw_actuator")
            print(f"claw_position: {claw_position}")

            joy_data = self.joy_instance.get_joy_data()
            print(f"joy_data: {joy_data}")

            print(f"goal_position: {goal_pos}")
            if joy_data is not None:
                if joy_data.buttons[4]==1:
                    v = float(claw_position[0])
                    v = min(v+50,100)
                    goal_pos[7] = v
                elif  joy_data.buttons[5]==1:
                    v = float(claw_position[0])
                    v = max(v-50,0)
                    goal_pos[7] = v
                else:
                    goal_pos[7] = float(claw_position[0])
            # Cap goal position when too far away from present position.
            # Slower fps expected due to reading from the follower.
            if self.config.max_relative_target is not None:
                present_pos = self.follower_arms[name].read("position")
                present_pos = torch.from_numpy(present_pos)
                goal_pos = ensure_safe_goal_position(goal_pos, present_pos, self.config.max_relative_target)

            # Used when record_data=True
            follower_goal_pos[name] = goal_pos
            print(f"goal pos {name}: {goal_pos}")
            
            goal_pos = goal_pos.numpy().astype(np.float32)
            self.follower_arms[name].write("position", goal_pos)
            self.logs[f"write_follower_{name}_goal_pos_dt_s"] = time.perf_counter() - before_fwrite_t

        #self.lumbar["main"].write("position", [0,0])
        # Early exit when recording data is not requested
        if not record_data:
            return

        # TODO(rcadene): Add velocity and other info
        # Read follower position
        follower_pos = {}
        for name in self.follower_arms:
            before_fread_t = time.perf_counter()
            follower_pos[name] = self.follower_arms[name].read("position")
            follower_pos[name] = torch.from_numpy(follower_pos[name])
            self.logs[f"read_follower_{name}_pos_dt_s"] = time.perf_counter() - before_fread_t

        # Create state by concatenating follower current position
        state = []
        for name in self.follower_arms:
            if name in follower_pos:
                state.append(follower_pos[name])
        state = torch.cat(state)

        # Create action by concatenating follower goal position
        action = []
        for name in self.follower_arms:
            if name in follower_goal_pos:
                action.append(follower_goal_pos[name])
        action = torch.cat(action)

        # Capture images from cameras

        # Populate output dictionaries
        obs_dict, action_dict = {}, {}
        obs_dict["observation.state"] = state
        action_dict["action"] = action

        return obs_dict, action_dict

    def capture_observation(self):
        """The returned observations do not have a batch dimension."""
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "ManipulatorRobot is not connected. You need to run `robot.connect()`."
            )

        # Read follower position
        follower_pos = {}
        for name in self.follower_arms:
            before_fread_t = time.perf_counter()
            follower_pos[name] = self.follower_arms[name].read("position")
            follower_pos[name] = torch.from_numpy(follower_pos[name])
            self.logs[f"read_follower_{name}_pos_dt_s"] = time.perf_counter() - before_fread_t

        # Create state by concatenating follower current position
        state = []
        for name in self.follower_arms:
            if name in follower_pos:
                state.append(follower_pos[name])
        state = torch.cat(state)

        # Capture images from cameras

        # Populate output dictionaries and format to pytorch
        obs_dict = {}
        obs_dict["observation.state"] = state
        #for name in self.cameras:
        #    obs_dict[f"observation.images.{name}"] = images[name]
        return obs_dict

    def send_action(self, action: torch.Tensor) -> torch.Tensor:
        """Command the follower arms to move to a target joint configuration.

        The relative action magnitude may be clipped depending on the configuration parameter
        `max_relative_target`. In this case, the action sent differs from original action.
        Thus, this function always returns the action actually sent.

        Args:
            action: tensor containing the concatenated goal positions for the follower arms.
        """
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "ManipulatorRobot is not connected. You need to run `robot.connect()`."
            )

        from_idx = 0
        to_idx = 0
        action_sent = []
        for name in self.follower_arms:
            # Get goal position of each follower arm by splitting the action vector
            to_idx += len(self.follower_arms[name].motor_names())
            goal_pos = action[from_idx:to_idx]
            from_idx = to_idx

            # Cap goal position when too far away from present position.
            # Slower fps expected due to reading from the follower.
            if self.config.max_relative_target is not None:
                present_pos = self.follower_arms[name].read("position")
                present_pos = torch.from_numpy(present_pos)
                goal_pos = ensure_safe_goal_position(goal_pos, present_pos, self.config.max_relative_target)

            # Save tensor to concat and return
            action_sent.append(goal_pos)

            # Send goal position to each follower
            goal_pos = goal_pos.numpy().astype(np.float32)
            self.follower_arms[name].write("position", goal_pos)

        return torch.cat(action_sent)

    def print_logs(self):
        pass
        # TODO(aliberts): move robot-specific logs logic here

    def disconnect(self):
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "ManipulatorRobot is not connected. You need to run `robot.connect()` before disconnecting."
            )
        agibotx1.stop_controller()

        self.is_connected = False

    def __del__(self):
        if getattr(self, "is_connected", False):
            self.disconnect()

    def get_motor_names(self, arm: dict[str, MotorsBus]) -> list:
        return [f"{arm}_{motor}" for arm, bus in arm.items() for motor in bus.motors]
    
    @property
    def motor_features(self) -> dict:
        action_names = self.get_motor_names(self.follower_arms)
        state_names = self.get_motor_names(self.follower_arms)
        return {
            "action": {
                "dtype": "float32",
                "shape": (len(action_names),),
                "names": action_names,
            },
            "observation.state": {
                "dtype": "float32",
                "shape": (len(state_names),),
                "names": state_names,
            },
        }    