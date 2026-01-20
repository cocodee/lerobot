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

    def update(self, frame, T_current):
        mode = frame["m"]

        # -----------------------------
        # Mode transition → set anchor
        # -----------------------------
        if mode != self.last_mode:
            if mode in ("POSITION", "ROTATE"):
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
        if mode == "POSITION":
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