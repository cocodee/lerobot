from dataclasses import dataclass

# 导入 LeRobot 相关的库
from lerobot.configs import parser
from lerobot.configs.policies import PreTrainedConfig

try:
    from lerobot.envs.configs import EnvConfig
except ImportError:
    pass # Keep it optional

@dataclass
class LocalInferenceConfig:
    """Configuration specific to local inference."""
    policy: PreTrainedConfig | None = None
    env: EnvConfig | None = None

    def __post_init__(self):
        policy_path = parser.get_path_arg("policy")
        if policy_path:
            cli_overrides = parser.get_cli_overrides("policy")
            self.policy = PreTrainedConfig.from_pretrained(policy_path, cli_overrides=cli_overrides)
            self.policy.pretrained_path = policy_path
        
        if self.policy is None:
            raise ValueError("LocalInferenceConfig requires a policy configuration.")
    @classmethod
    def __get_path_fields__(cls) -> list[str]:
        # Allows --local.policy=/path/to/policy from CLI
        return ["local.policy"]        

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
