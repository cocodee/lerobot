import numpy as np
from scipy.spatial.transform import Rotation as R


class DifferentialIKWrapper:
    """
    Wraps BeRobotKinematics to provide differential IK behavior.
    """

    def __init__(
        self,
        kinematics,
        pos_step=0.02,      # meters per step
        rot_step=0.1,       # rad per step
        max_pos_step=0.05,
        max_rot_step=0.3,
    ):
        self.kin = kinematics
        self.pos_step = pos_step
        self.rot_step = rot_step
        self.max_pos_step = max_pos_step
        self.max_rot_step = max_rot_step

    def _clamp_norm(self, v, max_norm):
        n = np.linalg.norm(v)
        if n < 1e-9:
            return v
        return v * min(1.0, max_norm / n)

    def step(self, q_current_deg, T_target):
        """
        q_current_deg: current joint positions (deg)
        T_target: absolute desired EE pose (4x4)
        """

        # 1. Current EE pose
        T_current = self.kin.forward_kinematics(q_current_deg)

        # -----------------------------
        # 2. Compute EE delta
        # -----------------------------

        # Position delta
        dp = T_target[:3, 3] - T_current[:3, 3]
        dp = self._clamp_norm(dp, self.max_pos_step)

        # Rotation delta (log map)
        R_cur = R.from_matrix(T_current[:3, :3])
        R_tgt = R.from_matrix(T_target[:3, :3])

        dR = R_tgt * R_cur.inv()
        drot = dR.as_rotvec()
        drot = self._clamp_norm(drot, self.max_rot_step)

        # -----------------------------
        # 3. Take a small step
        # -----------------------------
        T_step = np.eye(4)

        # Position step
        T_step[:3, 3] = T_current[:3, 3] + self.pos_step * dp

        # Orientation step
        R_step = R.from_rotvec(self.rot_step * drot) * R_cur
        T_step[:3, :3] = R_step.as_matrix()

        # -----------------------------
        # 4. Call original IK
        # -----------------------------
        q_next_deg = self.kin.inverse_kinematics(
            q_current_deg,
            T_step
        )

        return q_next_deg
