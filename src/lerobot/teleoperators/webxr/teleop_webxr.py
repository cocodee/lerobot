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
        match self.robot_type:
            case "sim_robot":
                return self.get_action_sim_robot()
            case "sim_robot_panda":
                return self.get_action_sim_robot_panda()
            case _:
                return self.get_action_sim_robot()
    def get_action_sim_robot(self) -> dict[str, Any]:
        """
        计算上一帧到当前帧的 Delta，并进行坐标系转换。
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self.name} is not connected.")

        # 如果没有数据，返回零动作
        if self.latest_data is None:
            return self._empty_action()

        # 1. 提取原始数据 (WebXR 坐标系: Y-Up, -Z Forward, X Right)
        raw_p = self.latest_data.get('p', [0, 0, 0])
        raw_q = self.latest_data.get('q', [0, 0, 0, 1]) # x,y,z,w
        gripper_state = self.latest_data.get('g', 1.0)

        # 2. 坐标系映射与缩放 (WebXR -> Robot)
        # 假设 Robot: Z-Up, X Forward, Y Left
        # 映射逻辑:
        #   WebXR -Z (前) -> Robot X (前)
        #   WebXR -X (左) -> Robot Y (左)  (注意：WebXR X是右，所以取反)
        #   WebXR  Y (上) -> Robot Z (上)
        # 红色=X轴(-Right), 绿色=Y轴(UP), 蓝色=Z轴(Forward)基座标系
        # (WebXR 坐标系: Y Up, Z -Forward, X Right)
        scale = self.config.pos_scale
        
        #curr_pos = np.array([
        #    -raw_p[2] * scale, # Robot X
        #    -raw_p[0] * scale, # Robot Y
        #     raw_p[1] * scale  # Robot Z
        #])

        curr_pos = np.array([
             -raw_p[0] * scale, # Robot X
             raw_p[1] * scale, # Robot Y
             -raw_p[2] * scale  # Robot Z
        ])
        # 处理旋转
        xr_rot = R.from_quat(raw_q) # WebXR 的原始旋转

        # 初始化上一帧
        if self.prev_pos is None:
            self.prev_pos = curr_pos
            self.prev_quat = xr_rot
            return self._empty_action()

        logger.info(f"[XR] pos: {curr_pos}, quat: {xr_rot}")
        # 3. 计算 Delta
        # 位置增量
        delta_pos = curr_pos - self.prev_pos
        logger.info(f"[XR] delta_pos: {delta_pos}")

        use_delta_rot = True
        if use_delta_rot:
            # 旋转增量 (Global Frame Delta): Q_delta = Q_curr * Q_prev_inv
            delta_rot_raw = xr_rot * self.prev_quat.inv()
            
            # 4. 旋转轴映射
            # 将 WebXR 坐标系的旋转变化映射到 Robot 坐标系
            rv = delta_rot_raw.as_rotvec()
            
            # 映射规则 (需要根据实际手感调整):
            # 绕 WebXR X轴转 (点头) -> Robot Y轴 (Pitch)
            # 绕 WebXR Y轴转 (摇头) -> Robot Z轴 (Yaw)
            # 绕 WebXR Z轴转 (歪头) -> Robot X轴 (Roll)
            # 注意方向符号
            mapped_rv = np.array([
                -rv[2], # Robot Roll (X)  <~ WebXR -Roll (Z)
                -rv[0], # Robot Pitch (Y) <~ WebXR -Pitch (X)
                 rv[1]  # Robot Yaw (Z)   <~ WebXR Yaw (Y)
            ])
            
            final_delta_rot = R.from_rotvec(mapped_rv)
            delta_quat = final_delta_rot.as_quat() # [x, y, z, w]
        else:
            R_base = R.from_matrix([
                [-1,  0,  0],
                [ 0,  1,  0],
                [ 0,  0, -1]
            ])
            
            # 计算在机器人坐标系下的绝对旋转: R_robot = R_base * R_xr
            final_rot = R_base * xr_rot
            curr_quat = final_rot.as_quat() # [x, y, z, w]
            delta_quat = curr_quat
            
        # 5. 更新状态
        self.prev_pos = curr_pos
        self.prev_quat = xr_rot

        # 6. 返回 Action 字典
        #return {
        #    "delta_x": float(delta_pos[0]),
        #    "delta_y": float(delta_pos[1]),
        #    "delta_z": float(delta_pos[2]),
        #    "delta_qx": float(delta_quat[0]),
        #    "delta_qy": float(delta_quat[1]),
        #    "delta_qz": float(delta_quat[2]),
        #    "delta_qw": float(delta_quat[3]),
        #    "gripper": float(gripper_state)
        #}
        return {
            "delta_x": float(delta_pos[0]),
            "delta_y": float(delta_pos[1]),
            "delta_z": float(delta_pos[2]),
            "delta_qx": float(delta_quat[0]),
            "delta_qy": float(delta_quat[1]),
            "delta_qz": float(delta_quat[2]),
            "delta_qw": float(delta_quat[3]),            
            "gripper": float(gripper_state)
        }

    def get_action_sim_robot_panda(self) -> dict[str, Any]:
        """
        计算上一帧到当前帧的 Delta，并进行坐标系转换。
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self.name} is not connected.")

        # 如果没有数据，返回零动作
        if self.latest_data is None:
            return self._empty_action()

        # 1. 提取原始数据 (WebXR 坐标系: Y-Up, -Z Forward, X Right)
        raw_p = self.latest_data.get('p', [0, 0, 0])
        raw_q = self.latest_data.get('q', [0, 0, 0, 1]) # x,y,z,w
        gripper_state = self.latest_data.get('g', 1.0)

        # 2. 坐标系映射与缩放 (WebXR -> Robot)
        # 假设 Robot: Z-Up, X Forward, Y Left
        # 映射逻辑:
        #   WebXR -Z (前) -> Robot X (前)
        #   WebXR -X (左) -> Robot Y (左)  (注意：WebXR X是右，所以取反)
        #   WebXR  Y (上) -> Robot Z (上)
        # 红色=X轴(-Right), 绿色=Y轴(UP), 蓝色=Z轴(Forward)基座标系
        # (WebXR 坐标系: Y Up, Z -Forward, X Right)
        scale = self.config.pos_scale
        
        #curr_pos = np.array([
        #    -raw_p[2] * scale, # Robot X
        #    -raw_p[0] * scale, # Robot Y
        #     raw_p[1] * scale  # Robot Z
        #])

        curr_pos = np.array([
             raw_p[1] * scale,# Robot X
             -raw_p[2] * scale,# Robot Y
             -raw_p[0] * scale,# Robot Z
        ])
        # 处理旋转
        xr_rot = R.from_quat(raw_q) # WebXR 的原始旋转

        # 初始化上一帧
        if self.prev_pos is None:
            self.prev_pos = curr_pos
            self.prev_quat = xr_rot
            return self._empty_action()

        logger.info(f"[XR] pos: {curr_pos}, quat: {xr_rot}")
        logger.info(f"[XR] curr_pos: {curr_pos}, prev_pos: {self.prev_pos}")
        logger.info(f"[XR] raw_p: {raw_p}, scale: {scale}")
        # 3. 计算 Delta
        # 位置增量
        delta_pos = curr_pos - self.prev_pos
        logger.info(f"[XR] delta_pos: {delta_pos}")

        use_delta_rot = True
        if use_delta_rot:
            # 旋转增量 (Global Frame Delta): Q_delta = Q_curr * Q_prev_inv
            delta_rot_raw = xr_rot * self.prev_quat.inv()

            # 4. 旋转轴映射
            # 将 WebXR 坐标系的旋转变化映射到 Robot 坐标系
            rv = delta_rot_raw.as_rotvec()

            # 映射规则 (根据实际手感调整):
            # 绕 WebXR X轴转 (前后倾斜) -> Robot Z轴
            # 绕 WebXR Y轴转 (左右倾斜) -> Robot Y轴 (Pitch)
            # 绕 WebXR Z轴转 (水平旋转) -> Robot X轴 (Roll)
            # 注意方向符号
            mapped_rv = np.array([
                -rv[2] ,  # Robot X (Roll)  <~ WebXR Z (水平旋转)
                -rv[0], # Robot Y (Pitch) <~ WebXR Y (左右倾斜)
                rv[1], # Robot Z (Yaw)   <~ WebXR X
            ])
            
            final_delta_rot = R.from_rotvec(mapped_rv)
            delta_quat = final_delta_rot.as_quat() # [x, y, z, w]
        else:
            R_base = R.from_matrix([
                [-1,  0,  0],
                [ 0,  1,  0],
                [ 0,  0, -1]
            ])
            
            # 计算在机器人坐标系下的绝对旋转: R_robot = R_base * R_xr
            final_rot = R_base * xr_rot
            curr_quat = final_rot.as_quat() # [x, y, z, w]
            delta_quat = curr_quat
            
        # 5. 更新状态
        self.prev_pos = curr_pos
        self.prev_quat = xr_rot

        # 6. 返回 Action 字典
        #return {
        #    "delta_x": float(delta_pos[0]),
        #    "delta_y": float(delta_pos[1]),
        #    "delta_z": float(delta_pos[2]),
        #    "delta_qx": float(delta_quat[0]),
        #    "delta_qy": float(delta_quat[1]),
        #    "delta_qz": float(delta_quat[2]),
        #    "delta_qw": float(delta_quat[3]),
        #    "gripper": float(gripper_state)
        #}
        return {
            "delta_x": float(delta_pos[0]),
            "delta_y": float(delta_pos[1]),
            "delta_z": float(delta_pos[2]),
            "delta_qx": float(delta_quat[0]),
            "delta_qy": float(delta_quat[1]),
            "delta_qz": float(delta_quat[2]),
            "delta_qw": float(delta_quat[3]),            
            "gripper": float(gripper_state)
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