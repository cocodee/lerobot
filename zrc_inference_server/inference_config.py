from dataclasses import dataclass

# 导入 LeRobot 相关的库
from lerobot.configs import parser
from lerobot.configs.policies import PreTrainedConfig

from typing import Optional

try:
    from lerobot.envs.configs import EnvConfig
except ImportError:
    pass # Keep it optional

@dataclass
class LocalInferenceConfig:
    """Configuration specific to local inference."""
    policy: PreTrainedConfig | None = None
    env: EnvConfig | None = None
    # 1. Add a dedicated field for the path. It's just a string.
    policy_path: Optional[str] = None
    
    # 2. Make the actual policy object not initializable from the constructor/CLI.
    #    We will create it ourselves in __post_init__.

    def __post_init__(self):
        # 3. Use the dedicated path field to load the policy object.
        if self.policy_path:
            # Here you can pass an empty dict for overrides, or decide how to handle them.
            self.policy = PreTrainedConfig.from_pretrained(self.policy_path, cli_overrides={})
        else:
            raise ValueError("LocalInferenceConfig requires a `policy_path` to be set.")  

@dataclass
class RemoteInferenceConfig:
    """Configuration specific to remote inference."""
    server_address: str = "127.0.0.1:8080"
    policy_type: str = "act"
    pretrained_name_or_path: str = "lerobot/aloha_sim_transfer_cube_human"
    policy_device: str = "cpu"
    actions_per_chunk: int = 50
    chunk_size_threshold: float = 0.5
    aggregate_fn_name: str = "weighted_average"
    verify_robot_cameras: bool = True
