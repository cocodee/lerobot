import logging
import threading
from abc import ABC, abstractmethod
from typing import Optional

# --- LeRobot Imports ---
from lerobot.datasets.utils import build_dataset_frame
from lerobot.policies.factory import make_policy
from lerobot.policies.factory import get_policy_class

from lerobot.robots import make_robot_from_config, RobotConfig
from lerobot.utils.control_utils import predict_action
from lerobot.utils.utils import get_safe_torch_device

# --- Remote Client Imports ---
from lerobot.scripts.server.configs import RobotClientConfig
from lerobot.scripts.server.robot_client import RobotClient

# --- Local Imports ---
from .inference_config import LocalInferenceConfig, RemoteInferenceConfig

logger = logging.getLogger('InferenceEngine')

class InferenceEngine(ABC):
    """Abstract base class for an inference engine."""
    def __init__(self, robot_config: RobotConfig, control_freq: int):
        self.robot_config = robot_config
        self.control_freq = control_freq
        self.robot = None

    @abstractmethod
    def setup(self, task_description: str):
        """Initialize resources for an inference task."""
        pass

    @abstractmethod
    def step(self):
        """Execute one step of the inference loop."""
        pass

    @abstractmethod
    def cleanup(self):
        """Release all resources."""
        pass

class LocalInferenceEngine(InferenceEngine):
    """Inference engine for running a local policy model."""
    def __init__(self, config: LocalInferenceConfig, robot_config: RobotConfig, control_freq: int):
        super().__init__(robot_config, control_freq)
        self.config = config
        self.policy = None
        self.task_description = ""

    def setup(self, task_description: str):
        logger.info("Setting up LocalInferenceEngine...")
        self.task_description = task_description
        
        # Instantiate and connect robot
        self.robot = make_robot_from_config(self.robot_config)
        self.robot.connect()
        logger.info("Robot connected.")

        # Instantiate policy
        #self.policy = make_policy(self.config.policy, ds_meta=None, env_cfg=self.config.env)
        policy_class = get_policy_class(self.config.policy.type)

        self.policy = policy_class.from_pretrained(self.config.policy_path)
        self.policy.to(self.config.policy.device)

        self.policy.reset()
        logger.info("Local policy instantiated.")
        
    def step(self):
        if not self.robot or not self.policy:
            raise RuntimeError("Engine not set up. Call setup() first.")
            
        observation = self.robot.get_observation()
        
        policy_meta_features = self.policy.config.input_features
        observation_frame = build_dataset_frame(policy_meta_features, observation, prefix="observation")
        
        action_values = predict_action(
            observation_frame, self.policy,
            get_safe_torch_device(self.policy.config.device), self.policy.config.use_amp,
            task=self.task_description, robot_type=self.robot.robot_type
        )
        
        action = {key: action_values[i].item() for i, key in enumerate(self.robot.action_features)}
        self.robot.send_action(action)

    def cleanup(self):
        logger.info("Cleaning up LocalInferenceEngine...")
        if self.robot and self.robot.is_connected:
            self.robot.disconnect()
            logger.info("Robot disconnected.")
        self.robot = None
        self.policy = None

class RemoteInferenceEngine(InferenceEngine):
    """Inference engine for connecting to a remote policy server."""
    def __init__(self, config: RemoteInferenceConfig, robot_config: RobotConfig, control_freq: int):
        super().__init__(robot_config, control_freq)
        self.config = config
        self.client: Optional[RobotClient] = None
        self.action_receiver_thread: Optional[threading.Thread] = None
        self.task_description = ""
        
    def setup(self, task_description: str):
        logger.info("Setting up RemoteInferenceEngine...")
        self.task_description = task_description
        
        client_config = RobotClientConfig(
            robot=self.robot_config,
            server_address=self.config.server_address,
            policy_type=self.config.policy_type,
            pretrained_name_or_path=self.config.pretrained_name_or_path,
            policy_device=self.config.policy_device,
            actions_per_chunk=self.config.actions_per_chunk,
            chunk_size_threshold=self.config.chunk_size_threshold,
            aggregate_fn_name=self.config.aggregate_fn_name,
            verify_robot_cameras=self.config.verify_robot_cameras,
            environment_dt=1.0 / self.control_freq,
            fps=self.control_freq, 
            task=task_description,
        )
        
        self.client = RobotClient(client_config)
        # The robot is inside the client, so we link it for consistency if needed.
        self.robot = self.client.robot
        
        if not self.client.start():
            raise RuntimeError("Failed to start RobotClient and connect to remote server.")
        
        self.action_receiver_thread = threading.Thread(target=self.client.receive_actions, daemon=True)
        self.action_receiver_thread.start()
        logger.info("Remote client and action receiver started.")

    def step(self):
        if not self.client:
            raise RuntimeError("Engine not set up. Call setup() first.")

        # Mimic the client's original control loop logic
        if self.client.actions_available():
            self.client.control_loop_action()
        if self.client._ready_to_send_observation():
            self.client.control_loop_observation(task=self.task_description)

    def cleanup(self):
        logger.info("Cleaning up RemoteInferenceEngine...")
        if self.client:
            self.client.stop()
        if self.action_receiver_thread and self.action_receiver_thread.is_alive():
            self.action_receiver_thread.join(timeout=1.0)
        self.client = None
        self.robot = None
        logger.info("Remote client stopped.")
