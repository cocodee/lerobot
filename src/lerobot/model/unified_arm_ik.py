import casadi
import meshcat.geometry as mg
import numpy as np
import pinocchio as pin
import time
from pinocchio import casadi as cpin
from pinocchio.visualize import MeshcatVisualizer
import os
import sys
import pickle
import logging
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple

# Logger setup
logger_mp = logging.getLogger(__name__)
parent2_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(parent2_dir)

from .weighted_moving_filter import WeightedMovingFilter

# ==========================================
# Configuration Data Class
# ==========================================

@dataclass
class EndEffectorConfig:
    parent_joint_name: str
    offset_translation: np.ndarray  # [x, y, z]
    offset_rotation: np.ndarray = field(default_factory=lambda: np.eye(3))

@dataclass
class IKWeights:
    translation: float = 50.0
    rotation: float = 1.0
    regularization: float = 0.02
    smooth: float = 0.1

@dataclass
class ArmIKConfig:
    name: str
    urdf_path: str
    model_dir: str
    cache_filename: str
    joints_to_lock: Optional[List[str]] = None  # Now optional, can be derived from active_joint_names
    active_joint_names: Optional[List[str]] = None # New parameter: explicitly list joints to keep active
    # 修改点：允许为 None，以支持单臂模式
    ee_left: Optional[EndEffectorConfig] = None
    ee_right: Optional[EndEffectorConfig] = None
    left_ee_frame_name: str = "L_ee"
    right_ee_frame_name: str = "R_ee"
    weights: IKWeights = field(default_factory=IKWeights)
    smooth_window_size: int = 14
    human_arm_length: float = 0.60
    robot_arm_length: float = 0.75

# ==========================================
# Unified IK Solver Class
# ==========================================

class UnifiedArmIK:
    def __init__(self, config: ArmIKConfig, visualization: bool = False):
        np.set_printoptions(precision=5, suppress=True, linewidth=200)
        self.config = config
        self.visualization = visualization
        
        # 1. 检查启用哪些手臂
        self.use_left = self.config.ee_left is not None
        self.use_right = self.config.ee_right is not None

        if not self.use_left and not self.use_right:
            raise ValueError(f"[{self.config.name}] Config Error: At least one arm (Left or Right) must be configured!")

        # Paths
        self.cache_path = self.config.cache_filename

        # Load Robot Model (Cache or URDF)
        if os.path.exists(self.cache_path) and (not self.visualization):
            logger_mp.info(f"[{self.config.name}] >>> Loading cached robot model: {self.cache_path}")
            self.robot, self.reduced_robot = self.load_cache()
        else:
            logger_mp.info(f"[{self.config.name}] >>> Loading URDF (slow)...")
            # Load full robot model first
            self.robot = pin.RobotWrapper.BuildFromURDF(self.config.urdf_path, self.config.model_dir)

            joints_to_lock = []
            if self.config.active_joint_names:
                # Calculate joints_to_lock based on active_joint_names
                all_joint_names = [self.robot.model.names[i] for i in range(self.robot.model.nq + 1)]
                joints_to_lock = [name for name in all_joint_names if name not in self.config.active_joint_names and name != "universe"]
            elif self.config.joints_to_lock:
                joints_to_lock = self.config.joints_to_lock
            else:
                # If neither is specified, default to locking no joints (all active except fixed ones)
                logger_mp.warning(f"[{self.config.name}] No active_joint_names or joints_to_lock specified. All non-fixed joints will be active.")
            
            # Build Reduced Robot
            self.reduced_robot = self.robot.buildReducedRobot(
                list_of_joints_to_lock=joints_to_lock,
                reference_configuration=np.array([0.0] * self.robot.model.nq),
            )
            
            # 2. 动态添加 EE Frames (只添加配置存在的)
            if self.use_left:
                self._add_ee_frame(self.config.left_ee_frame_name, self.config.ee_left)
            if self.use_right:
                self._add_ee_frame(self.config.right_ee_frame_name, self.config.ee_right)

            # Save Cache
            if not os.path.exists(self.cache_path):
                self.save_cache()
                logger_mp.info(f">>> Cache saved to {self.cache_path}")

        # CasADi Setup
        self._setup_casadi_optimization()

        # Smoothing Filter
        if config.smooth_window_size == 14:
            weights = np.array([0.4, 0.3, 0.2, 0.1])
        else:
             # Fallback simple weights
             weights = np.array([0.4, 0.3, 0.2, 0.1])
             
        self.smooth_filter = WeightedMovingFilter(weights, self.config.smooth_window_size)
        self.init_data = np.zeros(self.reduced_robot.model.nq)

        # Visualization
        self.vis = None
        if self.visualization:
            self._init_visualization()

    def _add_ee_frame(self, frame_name: str, ee_config: EndEffectorConfig):
        self.reduced_robot.model.addFrame(
            pin.Frame(
                frame_name,
                self.reduced_robot.model.getJointId(ee_config.parent_joint_name),
                pin.SE3(ee_config.offset_rotation, ee_config.offset_translation),
                pin.FrameType.OP_FRAME
            )
        )

    def _setup_casadi_optimization(self):
        # Model & Data
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()

        # --- 1. 定义 Opti 和 MX 变量 (外部接口保持不变) ---
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)

        # --- 2. 定义 SX 变量用于 Pinocchio 计算 (内部计算图) ---
        # 必须使用 SX 来满足 Pinocchio 的 C++ 接口要求
        q_sx = casadi.SX.sym("q_sx", self.reduced_robot.model.nq)
        q_last_sx = casadi.SX.sym("q_last_sx", self.reduced_robot.model.nq)

        # 使用 SX 变量运行运动学
        # cdata 会存储 SX 类型的表达式
        cpin.framesForwardKinematics(self.cmodel, self.cdata, q_sx)

        total_cost_sx = 0
        w = self.config.weights
        
        # 准备构建 Function 的输入列表
        sx_inputs = [q_sx, q_last_sx]
        mx_inputs = [self.var_q, self.var_q_last]

        # === 左臂逻辑 (基于 SX 构建) ===
        if self.use_left:
            # 定义 Opti 参数 (MX)
            self.param_tf_l = self.opti.parameter(4, 4)
            # 定义对应的 SX 符号
            tf_l_sx = casadi.SX.sym("tf_l_sx", 4, 4)
            
            # 添加到输入列表
            sx_inputs.append(tf_l_sx)
            mx_inputs.append(self.param_tf_l)

            self.L_hand_id = self.reduced_robot.model.getFrameId(self.config.left_ee_frame_name,cpin.FrameType.BODY)
            
            # 获取符号变量 (SX)
            pos_L = self.cdata.oMf[self.L_hand_id].translation
            rot_L = self.cdata.oMf[self.L_hand_id].rotation
            
            # 计算误差 (SX)
            diff_trans_L = pos_L - tf_l_sx[:3, 3]
            diff_rot_L = cpin.log3(rot_L @ tf_l_sx[:3, :3].T)
            
            # 累加 Cost (SX)
            total_cost_sx += w.translation * casadi.sumsqr(diff_trans_L)
            total_cost_sx += w.rotation * casadi.sumsqr(diff_rot_L)

        # === 右臂逻辑 (基于 SX 构建) ===
        if self.use_right:
            # 定义 Opti 参数 (MX)
            self.param_tf_r = self.opti.parameter(4, 4)
            # 定义对应的 SX 符号
            tf_r_sx = casadi.SX.sym("tf_r_sx", 4, 4)

            # 添加到输入列表
            sx_inputs.append(tf_r_sx)
            mx_inputs.append(self.param_tf_r)

            self.R_hand_id = self.reduced_robot.model.getFrameId(self.config.right_ee_frame_name,cpin.FrameType.BODY)
            
            # 获取符号变量 (SX)
            pos_R = self.cdata.oMf[self.R_hand_id].translation
            rot_R = self.cdata.oMf[self.R_hand_id].rotation
            
            # 计算误差 (SX)
            diff_trans_R = pos_R - tf_r_sx[:3, 3]
            diff_rot_R = cpin.log3(rot_R @ tf_r_sx[:3, :3].T)
            
            # 累加 Cost (SX)
            total_cost_sx += w.translation * casadi.sumsqr(diff_trans_R)
            total_cost_sx += w.rotation * casadi.sumsqr(diff_rot_R)

        # === 公共 Cost (SX) ===
        total_cost_sx += w.regularization * casadi.sumsqr(q_sx)
        total_cost_sx += w.smooth * casadi.sumsqr(q_sx - q_last_sx)

        # --- 3. 桥接 SX 和 MX ---
        # 创建一个 CasADi 函数，将 SX 计算图封装起来
        # 输入: [q, q_last, tf_l?, tf_r?] (SX)
        # 输出: cost (SX)
        cost_func = casadi.Function('cost_func', sx_inputs, [total_cost_sx])

        # 在 Opti 中调用该函数，传入 MX 变量，得到 MX 类型的 Cost
        total_cost_mx = cost_func(*mx_inputs)

        # --- 4. 设置优化器 ---
        self.opti.minimize(total_cost_mx)

        # 设置约束 (这里直接用 MX 变量即可，因为是简单的边界约束)
        self.opti.subject_to(self.opti.bounded(
            self.reduced_robot.model.lowerPositionLimit,
            self.var_q,
            self.reduced_robot.model.upperPositionLimit
        ))

        # Solver Options
        opts = {
            'expand': True, 'detect_simple_bounds': True, 'calc_lam_p': False, 'print_time': False,
            'ipopt.sb': 'yes', 'ipopt.print_level': 0, 'ipopt.max_iter': 30, 'ipopt.tol': 1e-4,
            'ipopt.acceptable_tol': 5e-4, 'ipopt.acceptable_iter': 5,
            'ipopt.warm_start_init_point': 'yes', 'ipopt.derivative_test': 'none',
            'ipopt.jacobian_approximation': 'exact'
        }
        self.opti.solver("ipopt", opts)

    def _init_visualization(self):
        self.vis = MeshcatVisualizer(self.reduced_robot.model, self.reduced_robot.collision_model, self.reduced_robot.visual_model)
        self.vis.initViewer(open=True)
        self.vis.loadViewerModel("pinocchio")
        
        # 4. 动态构建可视化 ID 列表
        frame_ids = []
        if self.use_left:
            frame_ids.append(self.reduced_robot.model.getFrameId(self.config.left_ee_frame_name))
        if self.use_right:
            frame_ids.append(self.reduced_robot.model.getFrameId(self.config.right_ee_frame_name))
            
        self.vis.displayFrames(True, frame_ids=frame_ids, axis_length=0.15, axis_width=5)
        self.vis.display(pin.neutral(self.reduced_robot.model))
        
        # 只显示启用的 Target Frame
        if self.use_left:
            self._setup_vis_target_frame(self.config.left_ee_frame_name + '_target')
        if self.use_right:
            self._setup_vis_target_frame(self.config.right_ee_frame_name + '_target')

    def _setup_vis_target_frame(self, frame_name):
        FRAME_AXIS_POSITIONS = np.array([[0,0,0], [1,0,0], [0,0,0], [0,1,0], [0,0,0], [0,0,1]]).astype(np.float32).T
        FRAME_AXIS_COLORS = np.array([[1,0,0], [1,0.6,0], [0,1,0], [0.6,1,0], [0,0,1], [0,0.6,1]]).astype(np.float32).T
        self.vis.viewer[frame_name].set_object(
            mg.LineSegments(
                mg.PointsGeometry(position=0.1 * FRAME_AXIS_POSITIONS, color=FRAME_AXIS_COLORS),
                mg.LineBasicMaterial(linewidth=10, vertexColors=True)
            )
        )

    def save_cache(self):
        data = {"robot_model": self.robot.model, "reduced_model": self.reduced_robot.model}
        with open(self.cache_path, "wb") as f:
            pickle.dump(data, f)

    def load_cache(self):
        with open(self.cache_path, "rb") as f:
            data = pickle.load(f)
        robot = pin.RobotWrapper()
        robot.model = data["robot_model"]
        robot.data = robot.model.createData()
        reduced_robot = pin.RobotWrapper()
        reduced_robot.model = data["reduced_model"]
        reduced_robot.data = reduced_robot.model.createData()
        return robot, reduced_robot

    def scale_arms(self, human_left_pose, human_right_pose):
        scale_factor = self.config.robot_arm_length / self.config.human_arm_length
        l_pose = human_left_pose.copy() if human_left_pose is not None else None
        r_pose = human_right_pose.copy() if human_right_pose is not None else None
        
        if l_pose is not None:
            l_pose[:3, 3] *= scale_factor
        if r_pose is not None:
            r_pose[:3, 3] *= scale_factor
        return l_pose, r_pose

    # 5. 修改 solve_ik 接口，允许输入 None
    def solve_ik(self, left_wrist=None, right_wrist=None, current_lr_arm_motor_q=None, current_lr_arm_motor_dq=None):
        if current_lr_arm_motor_q is not None:
            self.init_data = current_lr_arm_motor_q
        
        self.opti.set_initial(self.var_q, self.init_data)
        self.opti.set_value(self.var_q_last, self.init_data)

        # === 动态设置参数与可视化 ===
        if self.use_left:
            if left_wrist is None:
                logger_mp.warning("Left arm enabled in config but no pose provided! Skipping IK.")
                return self.init_data, np.zeros(self.reduced_robot.model.nv)
            
            self.opti.set_value(self.param_tf_l, left_wrist)
            if self.visualization:
                self.vis.viewer[self.config.left_ee_frame_name + '_target'].set_transform(left_wrist)

        if self.use_right:
            if right_wrist is None:
                logger_mp.warning("Right arm enabled in config but no pose provided! Skipping IK.")
                return self.init_data, np.zeros(self.reduced_robot.model.nv)
            
            self.opti.set_value(self.param_tf_r, right_wrist)
            if self.visualization:
                self.vis.viewer[self.config.right_ee_frame_name + '_target'].set_transform(right_wrist)

        try:
            self.opti.solve()
            sol_q = self.opti.value(self.var_q)
            
            self.smooth_filter.add_data(sol_q)
            sol_q = self.smooth_filter.filtered_data
            
            v = np.zeros(self.reduced_robot.model.nv) 
            self.init_data = sol_q
            sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))

            if self.visualization:
                self.vis.display(sol_q)

            return sol_q, sol_tauff

        except Exception as e:
            logger_mp.error(f"ERROR in convergence: {e}")
            sol_q = self.opti.debug.value(self.var_q)
            self.smooth_filter.add_data(sol_q)
            sol_q = self.smooth_filter.filtered_data
            
            if current_lr_arm_motor_q is not None:
                return current_lr_arm_motor_q, np.zeros(self.reduced_robot.model.nv)
            return sol_q, np.zeros(self.reduced_robot.model.nv)

# ==========================================
# Config Factory Functions
# ==========================================

def get_base_paths(unit_test: bool, asset_subdir: str, urdf_name: str) -> Tuple[str, str]:
    prefix = '../../' if unit_test else '../'
    model_dir = f"{prefix}assets/{asset_subdir}/"
    urdf_path = f"{model_dir}{urdf_name}"
    return urdf_path, model_dir

def create_g1_29_config(unit_test=False) -> ArmIKConfig:
    urdf, directory = get_base_paths(unit_test, "g1", "g1_body29_hand14.urdf")
    joints_to_lock = [
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
        "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint",
        "right_ankle_pitch_joint", "right_ankle_roll_joint",
        "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
        "left_hand_thumb_0_joint", "left_hand_thumb_1_joint", "left_hand_thumb_2_joint",
        "left_hand_middle_0_joint", "left_hand_middle_1_joint",
        "left_hand_index_0_joint", "left_hand_index_1_joint",
        "right_hand_thumb_0_joint", "right_hand_thumb_1_joint", "right_hand_thumb_2_joint",
        "right_hand_index_0_joint", "right_hand_index_1_joint",
        "right_hand_middle_0_joint", "right_hand_middle_1_joint"
    ]
    return ArmIKConfig(
        name="G1_29", urdf_path=urdf, model_dir=directory, cache_filename="g1_29_model_cache.pkl",
        joints_to_lock=joints_to_lock,
        ee_left=EndEffectorConfig("left_wrist_yaw_joint", np.array([0.05, 0, 0])),
        ee_right=EndEffectorConfig("right_wrist_yaw_joint", np.array([0.05, 0, 0])),
        weights=IKWeights(translation=50, rotation=1, regularization=0.02, smooth=0.1),
        smooth_window_size=14
    )

def create_g1_23_config(unit_test=False) -> ArmIKConfig:
    urdf, directory = get_base_paths(unit_test, "g1", "g1_body23.urdf")
    joints_to_lock = [
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
        "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint",
        "right_ankle_pitch_joint", "right_ankle_roll_joint", "waist_yaw_joint"
    ]
    return ArmIKConfig(
        name="G1_23", urdf_path=urdf, model_dir=directory, cache_filename="g1_23_model_cache.pkl",
        joints_to_lock=joints_to_lock,
        ee_left=EndEffectorConfig("left_wrist_roll_joint", np.array([0.20, 0, 0])),
        ee_right=EndEffectorConfig("right_wrist_roll_joint", np.array([0.20, 0, 0])),
        weights=IKWeights(translation=50, rotation=0.5, regularization=0.02, smooth=0.1),
        smooth_window_size=10
    )

def create_h1_2_config(unit_test=False) -> ArmIKConfig:
    urdf, directory = get_base_paths(unit_test, "h1_2", "h1_2.urdf")
    joints_to_lock = [
        "left_hip_yaw_joint", "left_hip_pitch_joint", "left_hip_roll_joint", "left_knee_joint",
        "left_ankle_pitch_joint", "left_ankle_roll_joint", "right_hip_yaw_joint", "right_hip_pitch_joint",
        "right_hip_roll_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
        "torso_joint",
        "L_index_proximal_joint", "L_index_intermediate_joint", "L_middle_proximal_joint", "L_middle_intermediate_joint",
        "L_pinky_proximal_joint", "L_pinky_intermediate_joint", "L_ring_proximal_joint", "L_ring_intermediate_joint",
        "L_thumb_proximal_yaw_joint", "L_thumb_proximal_pitch_joint", "L_thumb_intermediate_joint", "L_thumb_distal_joint",
        "R_index_proximal_joint", "R_index_intermediate_joint", "R_middle_proximal_joint", "R_middle_intermediate_joint",
        "R_pinky_proximal_joint", "R_pinky_intermediate_joint", "R_ring_proximal_joint", "R_ring_intermediate_joint",
        "R_thumb_proximal_yaw_joint", "R_thumb_proximal_pitch_joint", "R_thumb_intermediate_joint", "R_thumb_distal_joint"
    ]
    return ArmIKConfig(
        name="H1_2", urdf_path=urdf, model_dir=directory, cache_filename="h1_2_model_cache.pkl",
        joints_to_lock=joints_to_lock,
        ee_left=EndEffectorConfig("left_wrist_yaw_joint", np.array([0.05, 0, 0])),
        ee_right=EndEffectorConfig("right_wrist_yaw_joint", np.array([0.05, 0, 0])),
        weights=IKWeights(translation=50, rotation=1, regularization=0.02, smooth=0.1),
        smooth_window_size=14
    )

def create_h1_config(unit_test=False) -> ArmIKConfig:
    urdf, directory = get_base_paths(unit_test, "h1", "h1_with_hand.urdf")
    joints_to_lock = [
        "right_hip_roll_joint", "right_hip_pitch_joint", "right_knee_joint", "left_hip_roll_joint",
        "left_hip_pitch_joint", "left_knee_joint", "torso_joint", "left_hip_yaw_joint", "right_hip_yaw_joint",
        "left_ankle_joint", "right_ankle_joint",
        "L_index_proximal_joint", "L_index_intermediate_joint", "L_middle_proximal_joint", "L_middle_intermediate_joint",
        "L_ring_proximal_joint", "L_ring_intermediate_joint", "L_pinky_proximal_joint", "L_pinky_intermediate_joint",
        "L_thumb_proximal_yaw_joint", "L_thumb_proximal_pitch_joint", "L_thumb_intermediate_joint", "L_thumb_distal_joint",
        "R_index_proximal_joint", "R_index_intermediate_joint", "R_middle_proximal_joint", "R_middle_intermediate_joint",
        "R_ring_proximal_joint", "R_ring_intermediate_joint", "R_pinky_proximal_joint", "R_pinky_intermediate_joint",
        "R_thumb_proximal_yaw_joint", "R_thumb_proximal_pitch_joint", "R_thumb_intermediate_joint", "R_thumb_distal_joint",
        "left_hand_joint", "right_hand_joint"
    ]
    return ArmIKConfig(
        name="H1", urdf_path=urdf, model_dir=directory, cache_filename="h1_model_cache.pkl",
        joints_to_lock=joints_to_lock,
        ee_left=EndEffectorConfig("left_elbow_joint", np.array([0.2605 + 0.05, 0, 0])),
        ee_right=EndEffectorConfig("right_elbow_joint", np.array([0.2605 + 0.05, 0, 0])),
        weights=IKWeights(translation=50, rotation=0.5, regularization=0.02, smooth=0.1),
        smooth_window_size=8
    )

def create_panda_config(unit_test=False) -> ArmIKConfig:
    urdf, directory = get_base_paths(unit_test, "panda", "fr3.urdf")
    # For Panda, we typically don't lock joints since it's just a single arm
    # Users can specify active_joint_names if needed
    active_joint_names = [
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"
    ]
    return ArmIKConfig(
        name="Panda", urdf_path=urdf, model_dir=directory, cache_filename="panda_model_cache.pkl",
        joints_to_lock=None,  # No joints to lock by default for single arm
        active_joint_names=active_joint_names,
        ee_left=EndEffectorConfig("joint7", np.array([0.0, 0.0, 0.0])),  # end effector link
        ee_right=None,  # Panda is single arm
        left_ee_frame_name="hand",
        weights=IKWeights(translation=50, rotation=1.0, regularization=0.02, smooth=0.1),
        smooth_window_size=14
    )
# ==========================================
# UnifiedArmKinematics (BeRobotKinematics-like Interface)
# ==========================================

class UnifiedArmKinematics:
    def __init__(
        self,
        robot_type: str,
        target_frame_name: str,
        urdf_path: str,
        unit_test: bool = False,
        cache_filename: Optional[str] = None,
        joint_names: Optional[List[str]] = None,
        position_weight: float = 50.0,
        orientation_weight: float = 1.0,
        posture_weight: float = 0.02,
        posture_reference: Optional[np.ndarray] = None,
        velocity_limits: Optional[Dict[str, float]] = None,  # Not directly used by UnifiedArmIK (yet)
        joint_delta_limit: float = 0.1,
        visualization: bool = False,
        smooth_window_size: Optional[int] = None,
    ):
        self.target_frame_name = target_frame_name

        # Create ArmIKConfig using the helper function
        arm_ik_kwargs = {
            "position_weight": position_weight,
            "orientation_weight": orientation_weight,
            "posture_weight": posture_weight,
        }
        if smooth_window_size is not None:
            arm_ik_kwargs["smooth_window_size"] = smooth_window_size
        if joint_names is not None:
            arm_ik_kwargs["active_joint_names"] = joint_names

        arm_ik_config = create_unified_kinematics_config(
            robot_type=robot_type,
            urdf_path=urdf_path,
            target_frame_name=target_frame_name,
            unit_test=unit_test,
            cache_filename=cache_filename,
            **arm_ik_kwargs,
        )
        self.arm_ik_config = arm_ik_config
        self.unified_arm_ik = UnifiedArmIK(arm_ik_config, visualization=visualization)

        # Sanity check: Ensure target_frame_name exists in the IK config
        if self.target_frame_name not in [arm_ik_config.left_ee_frame_name, arm_ik_config.right_ee_frame_name]:
            raise ValueError(f"target_frame_name '{target_frame_name}' must be one of "
                             f"'{arm_ik_config.left_ee_frame_name}' or '{arm_ik_config.right_ee_frame_name}'")

        self.is_left_arm_control = (target_frame_name == arm_ik_config.left_ee_frame_name)

        # The active joints are now determined by the active_joint_names in arm_ik_config
        if self.unified_arm_ik.reduced_robot.model.nq > 0:
            self.joint_names = [self.unified_arm_ik.reduced_robot.model.names[i]
                                for i in range(1, self.unified_arm_ik.reduced_robot.model.nq + 1)]
        else:
            self.joint_names = []

        if joint_names is not None and len(joint_names) != len(self.joint_names):
            logger_mp.warning(
                f"Provided joint_names length ({len(joint_names)}) does not match "
                f"UnifiedArmIK controlled joints ({len(self.joint_names)}). "
                f"Using default joint names from UnifiedArmIK instead."
            )

        # The weights are now set when creating arm_ik_config, no need to update here.

        self.velocity_limits = velocity_limits  # Not directly used by UnifiedArmIK's CasADi solver
        self.joint_delta_limit = joint_delta_limit
        self.prev_joint_pos = None

        # Initializing prev_joint_pos for delta limiting
        if posture_reference is not None:
            if len(posture_reference) != len(self.joint_names):
                raise ValueError(f"posture_reference length ({len(posture_reference)}) does not match "
                                 f"controlled joint names length ({len(self.joint_names)})")
            self.prev_joint_pos = np.deg2rad(posture_reference)
        else:
            # Use neutral pose from Pinocchio as initial prev_joint_pos
            self.prev_joint_pos = pin.neutral(self.unified_arm_ik.reduced_robot.model).squeeze()

        # Set UnifiedArmIK's initial data to neutral or provided posture reference
        self.unified_arm_ik.init_data = self.prev_joint_pos


    def forward_kinematics(self, joint_pos_deg: np.ndarray) -> np.ndarray:
        """Standard Forward Kinematics"""
        if len(joint_pos_deg) != len(self.joint_names):
             raise ValueError(f"Input joint_pos_deg length ({len(joint_pos_deg)}) does not match "
                              f"expected joint names length ({len(self.joint_names)})")

        joint_pos_rad = np.deg2rad(joint_pos_deg)

        q = joint_pos_rad
        # Ensure we use the correct model for FK
        pin.framesForwardKinematics(self.unified_arm_ik.reduced_robot.model, self.unified_arm_ik.reduced_robot.data, q)

        frame_id = self.unified_arm_ik.reduced_robot.model.getFrameId(self.target_frame_name,cpin.FrameType.BODY)
        return self.unified_arm_ik.reduced_robot.data.oMf[frame_id].homogeneous

    def inverse_kinematics(
        self,
        current_joint_pos: np.ndarray,  # in degrees
        desired_ee_pose: np.ndarray,  # 4x4 homogeneous matrix
        joint_task_weight: Optional[float] = None,  # Not directly used by UnifiedArmIK's solve_ik
        target_joints: Optional[Dict[str, float]] = None,  # Not directly used by UnifiedArmIK
        position_weight: Optional[float] = None,
        orientation_weight: Optional[float] = None,
    ) -> np.ndarray:  # returns joint angles in degrees
        if len(current_joint_pos) != len(self.joint_names):
            raise ValueError(f"Input current_joint_pos length ({len(current_joint_pos)}) does not match "
                             f"expected joint names length ({len(self.joint_names)})")

        current_joint_rad = np.deg2rad(current_joint_pos)

        # Prepare target poses for UnifiedArmIK.solve_ik
        # The active arm gets desired_ee_pose. The inactive arm gets its current FK pose.
        # First, ensure data is updated with current_joint_rad for accurate inactive arm pose.
        pin.framesForwardKinematics(self.unified_arm_ik.reduced_robot.model, self.unified_arm_ik.reduced_robot.data, current_joint_rad)

        left_wrist_target = pin.SE3.Identity().homogeneous
        right_wrist_target = pin.SE3.Identity().homogeneous

        # Set active arm target
        if self.is_left_arm_control:
            left_wrist_target = desired_ee_pose
            # Inactive arm (right) target is its current pose to keep it stable
            right_ee_id = self.unified_arm_ik.reduced_robot.model.getFrameId(self.arm_ik_config.right_ee_frame_name)
            right_wrist_target = self.unified_arm_ik.reduced_robot.data.oMf[right_ee_id].homogeneous
        else:  # Controlling right arm
            right_wrist_target = desired_ee_pose
            # Inactive arm (left) target is its current pose to keep it stable
            left_ee_id = self.unified_arm_ik.reduced_robot.model.getFrameId(self.arm_ik_config.left_ee_frame_name)
            left_wrist_target = self.unified_arm_ik.reduced_robot.data.oMf[left_ee_id].homogeneous

        # Update weights dynamically if provided for this specific solve call
        # Note: UnifiedArmIK re-initializes CasADi solver only on UnifiedArmIK init.
        # For dynamic per-call weights, we'd need to reconfigure CasADi, which is heavy.
        # Sticking to weights set in UnifiedArmKinematics.__init__ for now, or those in ArmIKConfig.
        # However, we can update the weights on the UnifiedArmIK config object *before* calling solve_ik
        # and CasADi will use the latest values during its internal parameter binding.
        if position_weight is not None:
            self.unified_arm_ik.config.weights.translation = position_weight
        if orientation_weight is not None:
            self.unified_arm_ik.config.weights.rotation = orientation_weight
        if joint_task_weight is not None: # Map to regularization in UnifiedArmIK
            self.unified_arm_ik.config.weights.regularization = joint_task_weight

        # The `current_lr_arm_motor_q` corresponds to `init_data` in UnifiedArmIK.
        # It also serves as `var_q_last` for the smooth_cost.
        # We pass the current_joint_rad as initial guess and for smoothing reference.
        sol_q_rad, _ = self.unified_arm_ik.solve_ik(
            left_wrist_target,
            right_wrist_target,
            current_lr_arm_motor_q=current_joint_rad,
            current_lr_arm_motor_dq=np.zeros(current_joint_rad.shape)  # not used by UnifiedArmIK currently
        )

        # Apply joint_delta_limit (post-processing, similar to BeRobotKinematics's intent)
        if self.joint_delta_limit is not None and self.prev_joint_pos is not None:
            delta = sol_q_rad - self.prev_joint_pos
            delta = np.clip(delta, -self.joint_delta_limit, self.joint_delta_limit)
            sol_q_rad = self.prev_joint_pos + delta

        self.prev_joint_pos = sol_q_rad

        return np.rad2deg(sol_q_rad)

    def _get_default_posture(self) -> np.ndarray:
        """
        Returns the default posture (neutral pose) in degrees.
        This is mainly for API compatibility with BeRobotKinematics.
        """
        return np.rad2deg(pin.neutral(self.unified_arm_ik.reduced_robot.model).squeeze())


def create_unified_kinematics_config(
    robot_type: str,
    urdf_path: str,
    target_frame_name: str,
    unit_test: bool = False,
    cache_filename: Optional[str] = None,
    active_joint_names: Optional[List[str]] = None,
    **kwargs
) -> ArmIKConfig:
    """
    Helper function to create an ArmIKConfig for UnifiedArmKinematics based on robot type.
    """
    config_factory = {
        "g1_29": create_g1_29_config,
        "g1_23": create_g1_23_config,
        "h1_2": create_h1_2_config,
        "h1": create_h1_config,
        "panda": create_panda_config,
    }

    if robot_type not in config_factory:
        raise ValueError(f"Unsupported robot_type: {robot_type}. Choose from {list(config_factory.keys())}")

    base_config = config_factory[robot_type](unit_test=unit_test)

    # Override urdf_path and model_dir if provided
    if urdf_path is not None:
        base_config.urdf_path = urdf_path
        # Extract model_dir from urdf_path (directory containing the URDF)
        from pathlib import Path
        base_config.model_dir = str(Path(urdf_path).parent)

    # Override cache filename if provided
    if cache_filename is not None:
        base_config.cache_filename = cache_filename

    # Set active_joint_names if provided
    if active_joint_names is not None:
        base_config.active_joint_names = active_joint_names
        # Clear joints_to_lock if active_joint_names are set, to avoid conflict
        base_config.joints_to_lock = None

    # Apply kwargs to override specific EndEffectorConfig or IKWeights if needed
    if 'position_weight' in kwargs: base_config.weights.translation = kwargs.pop('position_weight')
    if 'orientation_weight' in kwargs: base_config.weights.rotation = kwargs.pop('orientation_weight')
    if 'posture_weight' in kwargs: base_config.weights.regularization = kwargs.pop('posture_weight')
    if 'smooth_window_size' in kwargs: base_config.smooth_window_size = kwargs.pop('smooth_window_size')

    # Override ee_left and ee_right frame names if provided. This is crucial for consistency with target_frame_name.
    # The base configs default to "L_ee" and "R_ee", so we should update ArmIKConfig's
    # `left_ee_frame_name` and `right_ee_frame_name` to match.
    if 'left_ee_frame_name' in kwargs: base_config.left_ee_frame_name = kwargs.pop('left_ee_frame_name')
    if 'right_ee_frame_name' in kwargs: base_config.right_ee_frame_name = kwargs.pop('right_ee_frame_name')

    if kwargs: # Check for any unhandled kwargs
        logger_mp.warning(f"Unhandled kwargs in create_unified_kinematics_config: {kwargs}")

    return base_config

# ==========================================
# Main Test Execution
# ==========================================

if __name__ == "__main__":
    UNIT_TEST = True
    VISUALIZATION = True
    
    # Select Robot Type and Target Frame Here for UnifiedArmKinematics
    robot_type = "h1_2" # Example: "g1_29", "g1_23", "h1_2", "h1"
    target_ee = "L_ee" # Example: "L_ee", "R_ee" (based on ArmIKConfig defaults)

    # 1. Instantiate UnifiedArmKinematics directly
    unified_kinematics_solver = UnifiedArmKinematics(
        robot_type=robot_type,
        target_frame_name=target_ee,
        unit_test=UNIT_TEST,
        cache_filename=f"{robot_type}_unified_kin_cache.pkl", # Custom cache name
        visualization=VISUALIZATION,
        # Example: Specify active joint names (e.g., for a single arm, or specific joints)
        # If not provided, all non-locked joints from the URDF will be used.
        # joint_names=["left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_elbow_joint"],
        # Other BeRobotKinematics-like parameters can be set here
        position_weight=100.0,
        orientation_weight=2.0,
        posture_weight=0.05,
        joint_delta_limit=np.deg2rad(10), # Limit joint change to 10 degrees per step
    )

    # Initial Position (neutral pose for the robot)
    initial_q_deg = np.rad2deg(pin.neutral(unified_kinematics_solver.unified_arm_ik.reduced_robot.model).squeeze())
    
    # Initial EE target (FK from initial_q)
    # This also populates the reduced_robot.data
    initial_ee_pose = unified_kinematics_solver.forward_kinematics(initial_q_deg)

    # Target Poses for the active arm
    L_tf_target_init = pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.25, +0.25, 0.1]))
    R_tf_target_init = pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.25, -0.25, 0.1]))

    # Set the initial target based on which arm is being controlled
    if unified_kinematics_solver.is_left_arm_control:
        active_ee_target = L_tf_target_init.homogeneous
    else:
        active_ee_target = R_tf_target_init.homogeneous

    # Simulation params
    rotation_speed = 0.005
    noise_trans = 0.001
    noise_rot = 0.01

    print(f"[{robot_type.upper()}] Initializing UnifiedArmKinematics for controlling {target_ee}...")

    if input("Enter 's' to start simulation:\n").lower() == 's':
        step = 0
        current_q_deg = initial_q_deg
        while True:
            # === Dynamic Target Pose Generation (simplified from original main block) ===
            rot_noise = pin.Quaternion(np.cos(np.random.normal(0, noise_rot)/2), 0, np.random.normal(0, noise_rot/2), 0).normalized()
            
            angle = rotation_speed * step if step <= 120 else rotation_speed * (240 - step)
            
            # Create a base rotation
            base_rot = pin.Quaternion(np.cos(angle/2), 0, np.sin(angle/2), 0)
            
            # Apply noise and then base rotation
            new_rotation = (rot_noise * base_rot).toRotationMatrix()

            # For translation, we'll make a simple oscillating movement
            z_offset = 0.1 * np.sin(step * 0.05)
            x_offset = 0.05 * np.cos(step * 0.05)

            current_target_translation = active_ee_target[:3, 3] + np.array([x_offset, 0, z_offset])
            
            # Update the active_ee_target's rotation and translation
            active_ee_target[:3, :3] = new_rotation
            active_ee_target[:3, 3] = current_target_translation

            # === Solve IK using UnifiedArmKinematics ===
            try:
                # current_q_deg from previous step's solution
                sol_q_deg = unified_kinematics_solver.inverse_kinematics(current_q_deg, active_ee_target)
                current_q_deg = sol_q_deg
                
                # Optional: Visualize the solution (UnifiedArmIK handles this internally if visualization=True)
                # To ensure the visualizer updates even if not explicitly solved by UnifiedArmIK's loop,
                # we can force display here using the underlying UnifiedArmIK instance.
                if VISUALIZATION:
                    unified_kinematics_solver.unified_arm_ik.vis.display(np.deg2rad(current_q_deg))

            except Exception as e:
                logger_mp.error(f"ERROR in UnifiedArmKinematics solve_ik: {e}")
                # On failure, keep previous joint positions
                pass

            step = (step + 1) % 241
            time.sleep(0.02)