from dataclasses import dataclass
from ..config import TeleoperatorConfig
import numpy as np
from typing import Optional, List, Union

@TeleoperatorConfig.register_subclass("webxr")
@dataclass
class WebxrTeleopConfig(TeleoperatorConfig):
    # Zenoh Router 连接地址
    zenoh_connect_key: str = "tcp/localhost:7447"
    # 订阅的 Topic
    zenoh_topic: str = "lerobot/webxr/teleop"
    # 移动灵敏度系数
    pos_scale: float = 1.0
    use_gripper: bool = True
    robot_type: str = "sim_robot"



@TeleoperatorConfig.register_subclass("webxr_delta")
@dataclass
class WebxrDeltaTeleopConfig(WebxrTeleopConfig):
    """Configuration for WebXR Delta teleoperator (same as WebXR, uses delta output)."""
    # WebXR 坐标系到机器人坐标系的旋转矩阵 (3x3)
    # 用于将 WebXR 控制器的方向映射到机器人坐标系
    # 示例 (Panda机器人):
    #   matrix = [[0, 1, 0],   # Robot X 来自 WebXR Y
    #             [0, 0, -1],  # Robot Y 来自 -WebXR X
    #             [-1, 0, 0]]  # Robot Z 来自 -WebXR Z
    xr_to_robot_matrix: Optional[List[List[float]]] = None

    # 轴映射旋转矩阵 (3x3) - 用于修正设备握持方向
    # 例如：手机竖着拿时需要将Y轴向上映射到机器人末端向前
    # 示例:
    #   mapping_matrix = [[0, 0, 1],  # Row 0
    #                     [1, 0, 0],  # Row 1
    #                     [0, 1, 0]]  # Row 2
    axis_map_matrix: Optional[List[List[float]]] = None