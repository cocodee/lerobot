"""
Panda 专用 Differential IK + Nullspace 实现

包装 BeRobotKinematics，实现稳定的差分控制
避免直接修改原有代码
"""

import numpy as np
from scipy.spatial.transform import Rotation as R


class PandaDifferentialIK:
    """
    Panda 专用 Differential IK + Nullspace
    依赖 BeRobotKinematics 提供：
      - robot (placo RobotWrapper)
      - joint_names
      - forward kinematics 已由外部完成
    """

    def __init__(
        self,
        kin,  # BeRobotKinematics instance
        dt: float = 0.02,
        damping: float = 1e-4,
        wrist_joint_name: str = "joint7",
        wrist_gain: float = 2.0,
        posture_gain: float = 0.2,
    ):
        self.kin = kin
        self.robot = kin.robot
        self.joint_names = kin.joint_names
        self.dt = dt
        self.damping = damping

        self.nq = len(self.joint_names)

        self.wrist_idx = self.joint_names.index(wrist_joint_name)
        self.wrist_gain = wrist_gain
        self.posture_gain = posture_gain

        # reference posture（rad）
        self.q_ref = kin.posture_reference.copy()

    # -------------------------
    # 工具函数
    # -------------------------

    def _get_joint_state(self):
        """获取当前关节状态"""
        return np.array([self.robot.get_joint(j) for j in self.joint_names])

    def _set_joint_state(self, q):
        """设置关节状态"""
        for i, j in enumerate(self.joint_names):
            self.robot.set_joint(j, float(q[i]))
        self.robot.update_kinematics()

    def _jacobian(self):
        """获取Jacobian矩阵"""
        # placo: world frame Jacobian of target frame
        J = self.robot.frame_jacobian(self.kin.target_frame_name)
        return J[:6, :]  # [vx, vy, vz, wx, wy, wz]

    def _damped_pinv(self, J):
        """阻尼伪逆"""
        JJt = J @ J.T
        return J.T @ np.linalg.inv(JJt + self.damping * np.eye(6))

    # -------------------------
    # 主接口
    # -------------------------

    def step(self, delta_pos, delta_rotvec):
        """
        执行一步差分IK

        Args:
            delta_pos: (3,) EE translation in base frame (meters)
            delta_rotvec: (3,) axis-angle (rad), base frame

        Returns:
            np.ndarray: 下一个关节角度（度）
        """
        # 当前关节
        q = self._get_joint_state()

        # Jacobian
        J = self._jacobian()
        J_pinv = self._damped_pinv(J)

        # task twist
        twist = np.zeros(6)
        twist[:3] = delta_pos / self.dt
        twist[3:] = delta_rotvec / self.dt

        # -------------------------
        # 主任务
        # -------------------------
        dq_task = J_pinv @ twist

        # -------------------------
        # Nullspace（关键）
        # -------------------------
        N = np.eye(self.nq) - J_pinv @ J

        dq_null = np.zeros(self.nq)

        # 1️⃣ wrist roll 稳定（核心）
        dq_null[self.wrist_idx] = -self.wrist_gain * (
            q[self.wrist_idx] - self.q_ref[self.wrist_idx]
        )

        # 2️⃣ posture 正则（轻）
        dq_null += -self.posture_gain * (q - self.q_ref)

        dq = dq_task + N @ dq_null

        # 积分
        q_next = q + dq * self.dt

        # 应用
        self._set_joint_state(q_next)

        return np.rad2deg(q_next)

    def set_reference_posture(self, q_ref):
        """设置参考姿态"""
        self.q_ref = q_ref.copy()

    def update_gains(self, wrist_gain=None, posture_gain=None):
        """更新控制增益"""
        if wrist_gain is not None:
            self.wrist_gain = wrist_gain
        if posture_gain is not None:
            self.posture_gain = posture_gain