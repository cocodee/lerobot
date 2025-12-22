from dataclasses import dataclass
from ..config import TeleoperatorConfig

@TeleoperatorConfig.register_subclass("webxr")
@dataclass
class WebxrTeleopConfig(TeleoperatorConfig):
    # Zenoh Router 连接地址
    zenoh_connect_key: str = "tcp/localhost:7447"
    # 订阅的 Topic
    zenoh_topic: str = "lerobot/webxr/teleop"
    # 移动灵敏度系数
    pos_scale: float = 2.5
    use_gripper: bool = True