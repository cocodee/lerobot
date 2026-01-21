import numpy as np
from scipy.spatial.transform import Rotation as R

class WebXRIntentTranslator:
    """
    WebXR → EE Target Pose (with Coordinate System Alignment)
    """

    def __init__(self, xr_to_robot_matrix=None):
        """
        xr_to_robot_matrix: (3, 3) or (4, 4) numpy array
                            描述 WebXR 坐标系到机械臂坐标系的旋转变换。
                            如果为 None，默认认为坐标系一致（Identity）。
        """
        self.anchor_pos = None
        self.anchor_rot = None
        self.anchor_xr_pos = None
        self.anchor_xr_rot = None

        self.last_mode = "IDLE"

        # 处理坐标系变换矩阵
        if xr_to_robot_matrix is None:
            self.R_align = R.identity()
        else:
            # 提取旋转部分 (3x3)
            self.R_align = R.from_matrix(xr_to_robot_matrix[:3, :3])
        
        # 缓存逆矩阵，用于旋转计算优化
        self.R_align_inv = self.R_align.inv()

    def _normalize_frame(self, frame):
        """
        将两种格式的 frame 标准化为内部使用的格式。
        支持的格式：
        1. 新格式：x, y, z, qx, qy, qz, qw, mode, type
        2. 旧格式：p, q, m, type (保持兼容性)
        """
        # 检查是否为新格式（包含 x, y, z 单独字段）
        if "x" in frame and "y" in frame and "z" in frame:
            # 新格式：转换为内部使用的 p 和 q 数组格式
            normalized = {
                "p": np.array([frame["x"], frame["y"], frame["z"]]),
                "q": np.array([frame["qx"], frame["qy"], frame["qz"], frame["qw"]]),
                "mode": frame.get("mode", "IDLE"),
                "type": frame.get("type", "webxr")
            }
        else:
            # 旧格式：直接使用（p, q, m, type）
            normalized = {
                "p": frame["p"],
                "q": frame["q"],
                "mode": frame.get("m", "IDLE"),
                "type": frame.get("type", "webxr")
            }
        return normalized

    def update(self, frame, T_current):
        # 标准化 frame 格式
        frame = self._normalize_frame(frame)
        mode = frame["mode"]

        # -----------------------------
        # Mode transition → set anchor
        # -----------------------------
        if mode != self.last_mode:
            if mode in ("TRANSLATE", "ROTATE"):
                self.anchor_pos = T_current[:3, 3].copy()
                self.anchor_rot = R.from_matrix(T_current[:3, :3])

                self.anchor_xr_pos = frame["p"].copy()
                self.anchor_xr_rot = R.from_quat(frame["q"])

        self.last_mode = mode

        if mode == "IDLE":
            return None

        # -----------------------------
        # POSITION mode
        # -----------------------------
        if mode == "TRANSLATE:
            # 1. 计算 WebXR 下的位移增量
            delta_xr = frame["p"] - self.anchor_xr_pos
            
            # 2. 【关键】将增量从 XR系 旋转到 机械臂系
            # 使用 scipy 的 apply 方法旋转向量
            delta_robot = self.R_align.apply(delta_xr)
            
            # 3. 叠加到机械臂锚点
            target_pos = self.anchor_pos + delta_robot

            T = np.eye(4)
            T[:3, :3] = self.anchor_rot.as_matrix()
            T[:3, 3] = target_pos
            return T

        # -----------------------------
        # ROTATE mode
        # -----------------------------
        if mode == "ROTATE":
            xr_rot = R.from_quat(frame["q"])
            
            # 1. 计算 WebXR 下的相对旋转 (Delta)
            # 这里的乘法顺序取决于你的定义，通常是 Global Frame 下的差值
            delta_rot_xr = xr_rot * self.anchor_xr_rot.inv()

            # 2. 【关键】将旋转增量变换到 机械臂系 (Basis Change)
            # 公式: R_new = M * R_old * M_inv
            # 这一步保证了：如果你绕 WebXR 的 Y 轴转，且 WebXR Y 对应 Robot Z，
            # 那么结果就是绕 Robot Z 轴转。
            delta_rot_robot = self.R_align * delta_rot_xr * self.R_align_inv

            # 3. 应用于机械臂锚点姿态
            target_rot = delta_rot_robot * self.anchor_rot

            T = np.eye(4)
            T[:3, :3] = target_rot.as_matrix()
            T[:3, 3] = self.anchor_pos
            return T

        return None