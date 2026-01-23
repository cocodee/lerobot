"""
WebXR 差分控制包装器

使用 Differential IK 替代传统的 Absolute IK
避免修改原有的 WebXRIntentTranslator
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

from .webxr_intent_translator import WebXRIntentTranslator
from .panda_differential_ik import PandaDifferentialIK
from lerobot.model.be_kinematics import BeRobotKinematics


class WebXRDifferentialWrapper:
    """
    WebXR 差分控制包装器

    包装 BeRobotKinematics 和 WebXRIntentTranslator
    使用 Differential IK 实现稳定的控制
    """

    def __init__(
        self,
        kin: BeRobotKinematics,
        dt: float = 0.02,
        max_delta_pos: float = 0.01,
        max_delta_rot: float = 0.1,
        damping: float = 1e-4,
        wrist_gain: float = 2.0,
        posture_gain: float = 0.2,
        wrist_joint_name: str = "joint7",
    ):
        """
        初始化差分控制包装器

        Args:
            kin: BeRobotKinematics 实例
            dt: 时间步长 (秒)
            max_delta_pos: 最大位置变化量 (米)
            max_delta_rot: 最大旋转变化量 (弧度)
            damping: Jacobian 阻尼系数
            wrist_gain: wrist 关节稳定增益
            posture_gain: 姿态正则化增益
            wrist_joint_name: wrist 关节名称
        """
        self.kin = kin
        self.dt = dt

        # 创建差分 IK
        self.diff_ik = PandaDifferentialIK(
            kin=kin,
            dt=dt,
            damping=damping,
            wrist_joint_name=wrist_joint_name,
            wrist_gain=wrist_gain,
            posture_gain=posture_gain,
        )

        # 创建 WebXR 意图翻译器（保持原有功能不变）
        self.intent_translator = WebXRIntentTranslator()

        # 限幅参数
        self.max_delta_pos = max_delta_pos
        self.max_delta_rot = max_delta_rot

        # 当前 EE 姿态
        self.T_current = None
        self.last_valid_pose = None

    def update(self, frame):
        """
        更新 WebXR 帧并返回控制结果

        Args:
            frame: WebXR 帧数据

        Returns:
            dict: 包含关节角度和控制状态的字典
        """
        # 使用 WebXRIntentTranslator 计算 target pose
        # 这里不直接应用 pose，而是计算差分
        T_target = self.intent_translator.update(frame, self.T_current)

        if T_target is None:
            # 没有有效目标，返回当前状态或保持不变
            if self.last_valid_pose is not None:
                return {
                    "joint_angles": self.last_valid_pose.copy(),
                    "status": "idle",
                    "delta_pos": np.zeros(3),
                    "delta_rot": np.zeros(3),
                }
            else:
                return {
                    "joint_angles": None,
                    "status": "no_target",
                    "delta_pos": np.zeros(3),
                    "delta_rot": np.zeros(3),
                }

        # 获取当前 EE 姿态
        if self.T_current is None:
            # 如果没有当前姿态，获取 FK 结果
            q_current = self._get_joint_state()
            self.T_current = self.kin.forward_kinematics(q_current)

        # -------------------------
        # 计算差分（关键改变）
        # -------------------------
        R_cur = R.from_matrix(self.T_current[:3, :3])
        p_cur = self.T_current[:3, 3]

        R_tgt = R.from_matrix(T_target[:3, :3])
        p_tgt = T_target[:3, 3]

        # differential error
        delta_pos = p_tgt - p_cur
        delta_rotvec = (R_tgt * R_cur.inv()).as_rotvec()

        # -------------------------
        # 限幅（极其重要）
        # -------------------------
        delta_pos = np.clip(delta_pos, -self.max_delta_pos, self.max_delta_pos)
        delta_rotvec = np.clip(delta_rotvec, -self.max_delta_rot, self.max_delta_rot)

        # -------------------------
        # 执行差分 IK
        # -------------------------
        try:
            q_next_deg = self.diff_ik.step(delta_pos, delta_rotvec)

            # 更新当前姿态
            self.T_current = self.kin.forward_kinematics(np.deg2rad(q_next_deg))

            # 保存有效姿态
            self.last_valid_pose = q_next_deg.copy()

            return {
                "joint_angles": q_next_deg,
                "status": "moving",
                "delta_pos": delta_pos,
                "delta_rot": delta_rotvec,
                "target_pos": p_tgt,
                "current_pos": p_cur,
            }

        except Exception as e:
            print(f"差分 IK 计算错误: {e}")
            return {
                "joint_angles": self.last_valid_pose,
                "status": "error",
                "delta_pos": np.zeros(3),
                "delta_rot": np.zeros(3),
                "error": str(e),
            }

    def _get_joint_state(self):
        """获取当前关节状态"""
        return np.array([self.kin.robot.get_joint(j) for j in self.kin.joint_names])

    def set_reference_posture(self, q_ref):
        """设置参考姿态"""
        self.diff_ik.set_reference_posture(q_ref)

    def update_gains(self, wrist_gain=None, posture_gain=None):
        """更新控制增益"""
        self.diff_ik.update_gains(wrist_gain, posture_gain)

    def get_limits(self):
        """获取当前限幅参数"""
        return {
            "max_delta_pos": self.max_delta_pos,
            "max_delta_rot": self.max_delta_rot,
        }

    def set_limits(self, max_delta_pos=None, max_delta_rot=None):
        """设置限幅参数"""
        if max_delta_pos is not None:
            self.max_delta_pos = max_delta_pos
        if max_delta_rot is not None:
            self.max_delta_rot = max_delta_rot