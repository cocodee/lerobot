import logging
import time
import json
import numpy as np
import zenoh
from typing import Any
from scipy.spatial.transform import Rotation as R

from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from ..teleoperator import Teleoperator
from .configuration_webxr import WebxrTeleopConfig
from lerobot.utils.utils import log_say


logger = logging.getLogger(__name__)

class WebxrTeleop(Teleoperator):
    """
    Teleop class to use WebXR (via Zenoh) for end effector control.
    """
    config_class = WebxrTeleopConfig
    name = "webxr"
    robot_type = "sim_robot"

    def __init__(self, config: WebxrTeleopConfig):
        super().__init__(config)
        self.config = config
        
        # Zenoh 相关状态
        self.session = None
        self.subscriber = None
        self.latest_data = None
        self.is_running = False

        # 状态追踪，用于计算 Delta
        self.prev_pos = None
        self.prev_quat = None
        if self.config.robot_type !=None and self.config.robot_type != "":
            self.robot_type = self.config.robot_type

    @property
    def action_features(self) -> dict:
        """
        定义输出动作的空间：
        - delta_x, delta_y, delta_z (位置增量)
        - delta_qx, delta_qy, delta_qz, delta_qw (旋转增量 - 四元数)
        - gripper (夹爪状态)
        总共 8 维
        """
        #return {
        #    "dtype": "float32",
        #    "shape": (8,),
        #    "names": {
        #        "delta_x": 0, "delta_y": 1, "delta_z": 2,
        #        "delta_qx": 3, "delta_qy": 4, "delta_qz": 5, "delta_qw": 6,
        #        "gripper": 7
        #    },
        #}
        if self.config.use_gripper:
            return {
                "dtype": "float32",
                "shape": (4,),
                "names": {"delta_x": 0, "delta_y": 1, "delta_z": 2, "gripper": 3},
            }
        else:
            return {
                "dtype": "float32",
                "shape": (3,),
                "names": {"delta_x": 0, "delta_y": 1, "delta_z": 2},
            }

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass    
    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.is_running

    @property
    def is_calibrated(self) -> bool:
        return True

    def connect(self) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self.name} is already connected.")

        logger.info(f"Connecting to Zenoh: {self.config.zenoh_connect_key}")
        
        # 配置 Zenoh
        zenoh_conf = zenoh.Config()
        config_json = {
            "mode": "client", 
            "connect": {"endpoints": [self.config.zenoh_connect_key]}
        }
        try:
            zenoh_conf = zenoh.Config.from_json5(json.dumps(config_json))
            self.session = zenoh.open(zenoh_conf)
            
            # 声明订阅者
            self.subscriber = self.session.declare_subscriber(
                self.config.zenoh_topic, 
                self._on_zenoh_data
            )
            self.is_running = True
            logger.info(f"WebXR Teleop connected and subscribed to {self.config.zenoh_topic}")
            
        except Exception as e:
            logger.error(f"Failed to connect to Zenoh: {e}")
            raise DeviceNotConnectedError(f"Could not connect to Zenoh: {e}")

    def _on_zenoh_data(self, sample):
        """Zenoh 回调函数"""
        try:
            # 兼容 Zenoh 新旧版本的 payload 解码
            if hasattr(sample.payload, 'to_bytes'):
                payload_bytes = sample.payload.to_bytes()
            else:
                payload_bytes = sample.payload
            
            data_str = payload_bytes.decode('utf-8')
            self.latest_data = json.loads(data_str)
        except Exception as e:
            logger.warning(f"Error parsing WebXR data: {e}")

    def calibrate(self) -> None:
        pass

    def configure(self):
        pass

    def get_action(self) -> dict[str, Any]:
        if self.latest_data is None:
            return self._empty_action()
        else:
            return {
                    "p": np.array(self.latest_data["p"], dtype=float),
                    "q": np.array(self.latest_data["q"], dtype=float),
                    "g": float(self.latest_data.get("g", 0.0)),
                    "m": self.latest_data.get("m", "IDLE"),
                    "type": "webxr"
            }
    def _empty_action(self):
        #return {
        #    "delta_x": 0.0, "delta_y": 0.0, "delta_z": 0.0,
        #    "delta_qx": 0.0, "delta_qy": 0.0, "delta_qz": 0.0, "delta_qw": 1.0, # Identity Quat
        #    "gripper": 1.0
        #}
        return {
            "delta_x": 0.0, "delta_y": 0.0, "delta_z": 0.0,
            "delta_qx": 0.0, "delta_qy": 0.0, "delta_qz": 0.0, "delta_qw": 1.0, # Identity Quat
            "gripper": 0.0
        }

    def disconnect(self) -> None:
        if self.is_connected:
            if self.session:
                self.session.close()
            self.is_running = False
            logger.info(f"{self.name} disconnected.")
        else:
            # 模仿 KeyboardTeleop 的行为
            raise DeviceNotConnectedError(f"{self.name} is not connected.")

    def reset(self) -> None:
        """Resets the teleoperator state."""
        self.prev_pos = None
        self.prev_quat = None
        self.latest_data = None
        log_say(f"{self.name} teleoperator state reset.")
        logger.info(f"{self.name} teleoperator state reset.")