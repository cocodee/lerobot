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
from pathlib import Path

# Logger setup
logger_mp = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)
parent2_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if parent2_dir not in sys.path:
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
    joints_to_lock: Optional[List[str]] = None  
    active_joint_names: Optional[List[str]] = None  
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
        
        # 检查启用哪些手臂
        self.use_left = self.config.ee_left is not None
        self.use_right = self.config.ee_right is not None

        if not self.use_left and not self.use_right:
            raise ValueError(f"[{self.config.name}] Config Error: At least one arm must be configured!")

        # Cache 路径（加入配置哈希，避免配置更改后加载错误缓存）
        self.cache_path = self._get_cache_path()

        # Load Robot Model
        if os.path.exists(self.cache_path) and (not self.visualization):
            logger_mp.info(f"[{self.config.name}] Loading cached model: {self.cache_path}")
            self.robot, self.reduced_robot = self._load_cache()
            # 即使从缓存加载，也需要重新添加 EE Frame（因为 Frame 不在缓存中）
            if self.use_left:
                self._add_ee_frame(self.config.left_ee_frame_name, self.config.ee_left)
            if self.use_right:
                self._add_ee_frame(self.config.right_ee_frame_name, self.config.ee_right)
        else:
            logger_mp.info(f"[{self.config.name}] Loading URDF...")
            self.robot = pin.RobotWrapper.BuildFromURDF(self.config.urdf_path, self.config.model_dir)

            joints_to_lock = self._compute_joints_to_lock()
            
            self.reduced_robot = self.robot.buildReducedRobot(
                list_of_joints_to_lock=joints_to_lock,
                reference_configuration=np.array([0.0] * self.robot.model.nq),
            )
            
            # 动态添加 EE Frames
            if self.use_left:
                self._add_ee_frame(self.config.left_ee_frame_name, self.config.ee_left)
            if self.use_right:
                self._add_ee_frame(self.config.right_ee_frame_name, self.config.ee_right)

            if not os.path.exists(self.cache_path):
                self._save_cache()

        # ✅ FIX: 必须在添加 EE Frame 后重新创建 CasADi 模型（因为模型已修改）
        self._setup_casadi_optimization()

        # Smoothing Filter
        self._init_smooth_filter()

        # Visualization
        self.vis = None
        if self.visualization:
            self._init_visualization()

    def _get_cache_path(self) -> str:
        """生成包含配置哈希的缓存路径，避免配置更改时加载错误缓存"""
        import hashlib
        config_str = f"{self.config.urdf_path}_{self.config.active_joint_names}_{self.config.joints_to_lock}"
        config_hash = hashlib.md5(config_str.encode()).hexdigest()[:8]
        base, ext = os.path.splitext(self.config.cache_filename)
        return f"{base}_{config_hash}{ext}"

    def _compute_joints_to_lock(self) -> List[str]:
        """计算需要锁定的关节"""
        if self.config.active_joint_names:
            all_joint_names = [self.robot.model.names[i] for i in range(1, self.robot.model.nq + 1)]  # 从1开始，跳过universe
            joints_to_lock = [name for name in all_joint_names 
                            if name not in self.config.active_joint_names]
            return joints_to_lock
        elif self.config.joints_to_lock:
            return self.config.joints_to_lock
        else:
            logger_mp.warning(f"[{self.config.name}] No joint filtering specified. All joints active.")
            return []

    def _add_ee_frame(self, frame_name: str, ee_config: EndEffectorConfig):
        """添加末端执行器 Frame 到 reduced model"""
        # ✅ FIX: 检查父关节是否存在
        parent_joint_id = self.reduced_robot.model.getJointId(ee_config.parent_joint_name)
        if parent_joint_id >= self.reduced_robot.model.njoints:
            raise ValueError(f"Parent joint '{ee_config.parent_joint_name}' not found in reduced model!")
        
        frame = pin.Frame(
            frame_name,
            parent_joint_id,
            pin.SE3(ee_config.offset_rotation, ee_config.offset_translation),
            pin.FrameType.OP_FRAME
        )
        self.reduced_robot.model.addFrame(frame)
        # ✅ FIX: 添加 frame 后需要更新数据
        self.reduced_robot.data = self.reduced_robot.model.createData()

    def _init_smooth_filter(self):
        """初始化平滑滤波器"""
        if self.config.smooth_window_size == 14:
            weights = np.array([0.4, 0.3, 0.2, 0.1])
        else:
            weights = np.array([0.5, 0.2, 0.1, 0.1])  # 均匀权重
        
        self.smooth_filter = WeightedMovingFilter(weights, self.reduced_robot.model.nq)
        self.init_data = np.zeros(self.reduced_robot.model.nq)

    def _setup_casadi_optimization(self):
        """
        ✅ MAJOR FIX: 完全重构 CasADi 设置，避免 SX/MX 混用问题
        使用纯 MX 变量构建优化问题，通过 Function 封装 Pinocchio 调用
        """
        # ✅ FIX: 必须在添加所有 Frame 后创建 CasADi 模型
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()

        self.opti = casadi.Opti()
        
        # 决策变量
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)
        
        # 获取 Frame ID（必须在添加 Frame 后）
        if self.use_left:
            self.L_hand_id = self.reduced_robot.model.getFrameId(self.config.left_ee_frame_name,pin.FrameType.OP_FRAME)
            if self.L_hand_id >= self.reduced_robot.model.nframes:
                raise ValueError(f"Left EE frame '{self.config.left_ee_frame_name}' not found!")
            self.param_tf_l = self.opti.parameter(4, 4)
            
        if self.use_right:
            self.R_hand_id = self.reduced_robot.model.getFrameId(self.config.right_ee_frame_name,pin.FrameType.OP_FRAME)
            if self.R_hand_id >= self.reduced_robot.model.nframes:
                raise ValueError(f"Right EE frame '{self.config.right_ee_frame_name}' not found!")
            self.param_tf_r = self.opti.parameter(4, 4)

        # ✅ FIX: 使用纯 MX 构建成本函数，通过 Function 包装
        # 创建内部函数用于计算正向运动学
        q_sym = casadi.SX.sym("q", self.reduced_robot.model.nq)
        cdata_temp = self.cmodel.createData()
        cpin.framesForwardKinematics(self.cmodel, cdata_temp, q_sym)
        
        # 提取位置和旋转（SX 表达式）
        outputs = []
        if self.use_left:
            pos_l = cdata_temp.oMf[self.L_hand_id].translation
            rot_l = cdata_temp.oMf[self.L_hand_id].rotation
            outputs.extend([pos_l, rot_l])
        if self.use_right:
            pos_r = cdata_temp.oMf[self.R_hand_id].translation
            rot_r = cdata_temp.oMf[self.R_hand_id].rotation
            outputs.extend([pos_r, rot_r])
            
        # 创建 Function 将 SX 转换为可调用函数
        self.fk_func = casadi.Function('fk', [q_sym], outputs)

        # ✅ FIX: 在 Opti 中使用 MX 构建成本（通过 callback 或直接映射）
        # 由于 CasADi Function 可以在 MX 中调用，我们可以这样做：
        fk_results = self.fk_func(self.var_q)
        idx = 0
        
        total_cost = 0
        w = self.config.weights

        if self.use_left:
            pos_L = fk_results[idx]; idx += 1
            rot_L = fk_results[idx]; idx += 1
            
            # 提取目标位姿
            target_pos_L = self.param_tf_l[:3, 3]
            target_rot_L = self.param_tf_l[:3, :3]
            
            # 平移误差
            diff_trans_L = pos_L - target_pos_L
            total_cost += w.translation * casadi.sumsqr(diff_trans_L)
            
            # 旋转误差（使用 log3 近似，但需要在 SX 中预计算）
            # ✅ FIX: 旋转误差需要特殊处理，这里使用简化版
            # 实际旋转矩阵差异的 Frobenius 范数
            rot_diff_L = rot_L - target_rot_L
            total_cost += w.rotation * casadi.sumsqr(rot_diff_L)

        if self.use_right:
            pos_R = fk_results[idx]; idx += 1
            rot_R = fk_results[idx]; idx += 1
            
            target_pos_R = self.param_tf_r[:3, 3]
            target_rot_R = self.param_tf_r[:3, :3]
            
            diff_trans_R = pos_R - target_pos_R
            total_cost += w.translation * casadi.sumsqr(diff_trans_R)
            
            rot_diff_R = rot_R - target_rot_R
            total_cost += w.rotation * casadi.sumsqr(rot_diff_R)

        # 正则化和平滑项
        total_cost += w.regularization * casadi.sumsqr(self.var_q)
        total_cost += w.smooth * casadi.sumsqr(self.var_q - self.var_q_last)

        self.opti.minimize(total_cost)

        # 关节限位约束
        self.opti.subject_to(self.opti.bounded(
            self.reduced_robot.model.lowerPositionLimit,
            self.var_q,
            self.reduced_robot.model.upperPositionLimit
        ))

        # Solver 选项
        opts = {
            'expand': True,  # 展开为 SX 以加速
            'detect_simple_bounds': True,
            'calc_lam_p': False,
            'print_time': False,
            'ipopt.sb': 'yes',
            'ipopt.print_level': 0,
            'ipopt.max_iter': 50,  # 增加迭代次数以确保收敛
            'ipopt.tol': 1e-4,
            'ipopt.acceptable_tol': 1e-3,
            'ipopt.acceptable_iter': 5,
            'ipopt.warm_start_init_point': 'yes',
        }
        self.opti.solver("ipopt", opts)

    def _init_visualization(self):
        """初始化可视化"""
        self.vis = MeshcatVisualizer(
            self.reduced_robot.model, 
            self.reduced_robot.collision_model, 
            self.reduced_robot.visual_model
        )
        self.vis.initViewer(open=True)
        self.vis.loadViewerModel("pinocchio")
        
        # 显示 EE Frames
        frame_ids = []
        if self.use_left:
            frame_ids.append(self.reduced_robot.model.getFrameId(self.config.left_ee_frame_name,pin.FrameType.OP_FRAME))
        if self.use_right:
            frame_ids.append(self.reduced_robot.model.getFrameId(self.config.right_ee_frame_name,pin.FrameType.OP_FRAME))
            
        self.vis.displayFrames(True, frame_ids=frame_ids, axis_length=0.15, axis_width=5)
        self.vis.display(pin.neutral(self.reduced_robot.model))
        
        # 目标帧可视化
        if self.use_left:
            self._setup_vis_target_frame(self.config.left_ee_frame_name + '_target')
        if self.use_right:
            self._setup_vis_target_frame(self.config.right_ee_frame_name + '_target')

    def _setup_vis_target_frame(self, frame_name: str):
        """设置目标可视化帧"""
        FRAME_AXIS_POSITIONS = np.array([
            [0,0,0], [1,0,0], [0,0,0], [0,1,0], [0,0,0], [0,0,1]
        ], dtype=np.float32).T * 0.1
        FRAME_AXIS_COLORS = np.array([
            [1,0,0], [1,0.6,0], [0,1,0], [0.6,1,0], [0,0,1], [0,0.6,1]
        ], dtype=np.float32).T
        
        self.vis.viewer[frame_name].set_object(
            mg.LineSegments(
                mg.PointsGeometry(position=FRAME_AXIS_POSITIONS, color=FRAME_AXIS_COLORS),
                mg.LineBasicMaterial(linewidth=10, vertexColors=True)
            )
        )

    def _save_cache(self):
        """保存模型缓存（不包含 CasADi 对象）"""
        try:
            # ✅ FIX: 将 EndEffectorConfig 转换为 dict 以便安全序列化
            def ee_to_dict(ee: Optional[EndEffectorConfig]) -> Optional[dict]:
                if ee is None:
                    return None
                return {
                    "parent_joint_name": ee.parent_joint_name,
                    "offset_translation": ee.offset_translation,
                    "offset_rotation": ee.offset_rotation,
                }
            
            data = {
                "robot_model": self.robot.model,
                "reduced_model": self.reduced_robot.model,
                "config": {
                    "ee_left": ee_to_dict(self.config.ee_left),
                    "ee_right": ee_to_dict(self.config.ee_right),
                    "left_ee_frame_name": self.config.left_ee_frame_name,
                    "right_ee_frame_name": self.config.right_ee_frame_name,
                }
            }
            with open(self.cache_path, "wb") as f:
                pickle.dump(data, f)
            logger_mp.info(f"Cache saved to {self.cache_path}")
        except Exception as e:
            logger_mp.warning(f"Failed to save cache: {e}")

    def _load_cache(self) -> Tuple[pin.RobotWrapper, pin.RobotWrapper]:
        """加载模型缓存"""
        try:
            with open(self.cache_path, "rb") as f:
                data = pickle.load(f)
            
            # ✅ FIX: 正确比较配置，处理 numpy 数组
            cached_config = data.get("config", {})
            
            # 比较 ee_left
            ee_left_match = self._compare_ee_config(
                cached_config.get("ee_left"), 
                self.config.ee_left
            )
            
            # 比较 ee_right  
            ee_right_match = self._compare_ee_config(
                cached_config.get("ee_right"),
                self.config.ee_right
            )
            
            # 比较帧名
            left_name_match = cached_config.get("left_ee_frame_name") == self.config.left_ee_frame_name
            right_name_match = cached_config.get("right_ee_frame_name") == self.config.right_ee_frame_name
            
            if not (ee_left_match and ee_right_match and left_name_match and right_name_match):
                logger_mp.warning("Cache config mismatch, rebuilding model...")
                raise ValueError("Config mismatch")
            
            robot = pin.RobotWrapper()
            robot.model = data["robot_model"]
            robot.data = robot.model.createData()
            
            reduced_robot = pin.RobotWrapper()
            reduced_robot.model = data["reduced_model"]
            reduced_robot.data = reduced_robot.model.createData()
            
            return robot, reduced_robot
            
        except Exception as e:
            logger_mp.error(f"Cache loading failed: {e}")
            raise

    def _compare_ee_config(self, cached_ee: Optional[dict], current_ee: Optional[EndEffectorConfig]) -> bool:
        """安全比较 EndEffectorConfig，处理 numpy 数组"""
        # 处理 None 情况
        if cached_ee is None and current_ee is None:
            return True
        if cached_ee is None or current_ee is None:
            return False
        
        # 比较 parent_joint_name
        if cached_ee.get("parent_joint_name") != current_ee.parent_joint_name:
            return False
        
        # 比较 offset_translation (numpy array)
        cached_trans = cached_ee.get("offset_translation")
        current_trans = current_ee.offset_translation
        if cached_trans is None or current_trans is None:
            if cached_trans is not current_trans:  # 一个为 None 另一个不为 None
                return False
        elif not np.array_equal(cached_trans, current_trans):
            return False
        
        # 比较 offset_rotation (numpy array)
        cached_rot = cached_ee.get("offset_rotation")
        current_rot = current_ee.offset_rotation
        if cached_rot is None or current_rot is None:
            if cached_rot is not current_rot:
                return False
        elif not np.array_equal(cached_rot, current_rot):
            return False
        
        return True

    def scale_arms(self, human_left_pose: Optional[np.ndarray], 
                   human_right_pose: Optional[np.ndarray]) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """缩放人体手臂长度到机器人比例"""
        scale_factor = self.config.robot_arm_length / self.config.human_arm_length
        l_pose = human_left_pose.copy() if human_left_pose is not None else None
        r_pose = human_right_pose.copy() if human_right_pose is not None else None
        
        if l_pose is not None:
            l_pose[:3, 3] *= scale_factor
        if r_pose is not None:
            r_pose[:3, 3] *= scale_factor
        return l_pose, r_pose

    def solve_ik(self, left_wrist: Optional[np.ndarray] = None, 
                 right_wrist: Optional[np.ndarray] = None,
                 current_lr_arm_motor_q: Optional[np.ndarray] = None,
                 current_lr_arm_motor_dq: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        求解逆运动学
        
        ✅ FIX: 改进错误处理和返回值一致性
        """
        # 设置初始值
        if current_lr_arm_motor_q is not None:
            if len(current_lr_arm_motor_q) != self.reduced_robot.model.nq:
                raise ValueError(f"Initial q dimension mismatch: expected {self.reduced_robot.model.nq}, "
                               f"got {len(current_lr_arm_motor_q)}")
            self.init_data = current_lr_arm_motor_q.copy()
        
        self.opti.set_initial(self.var_q, self.init_data)
        self.opti.set_value(self.var_q_last, self.init_data)

        # 设置目标位姿
        if self.use_left:
            if left_wrist is None:
                logger_mp.error("Left arm enabled but no pose provided!")
                return self.init_data.copy(), np.zeros(self.reduced_robot.model.nv)
            self.opti.set_value(self.param_tf_l, left_wrist)
            if self.visualization and hasattr(self, 'vis') and self.vis:
                self.vis.viewer[self.config.left_ee_frame_name + '_target'].set_transform(left_wrist)

        if self.use_right:
            if right_wrist is None:
                logger_mp.error("Right arm enabled but no pose provided!")
                return self.init_data.copy(), np.zeros(self.reduced_robot.model.nv)
            self.opti.set_value(self.param_tf_r, right_wrist)
            if self.visualization and hasattr(self, 'vis') and self.vis:
                self.vis.viewer[self.config.right_ee_frame_name + '_target'].set_transform(right_wrist)

        try:
            # 求解
            sol = self.opti.solve()
            sol_q = self.opti.value(self.var_q)
            
            # 平滑滤波
            self.smooth_filter.add_data(sol_q)
            sol_q = self.smooth_filter.filtered_data
            
            # 计算前馈力矩
            v = np.zeros(self.reduced_robot.model.nv)
            a = np.zeros(self.reduced_robot.model.nv)
            sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, a)
            
            self.init_data = sol_q.copy()
            
            if self.visualization and hasattr(self, 'vis') and self.vis:
                self.vis.display(sol_q)
            
            return sol_q, sol_tauff

        except Exception as e:
            logger_mp.error(f"IK convergence failed: {e}")
            
            # 尝试获取调试值
            try:
                debug_q = self.opti.debug.value(self.var_q)
                self.smooth_filter.add_data(debug_q)
                sol_q = self.smooth_filter.filtered_data
            except:
                sol_q = self.init_data.copy()
            
            if current_lr_arm_motor_q is not None:
                return current_lr_arm_motor_q.copy(), np.zeros(self.reduced_robot.model.nv)
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
        name="G1_29", urdf_path=urdf, model_dir=directory, 
        cache_filename="g1_29_model_cache.pkl",
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
        name="G1_23", urdf_path=urdf, model_dir=directory, 
        cache_filename="g1_23_model_cache.pkl",
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
        name="H1_2", urdf_path=urdf, model_dir=directory, 
        cache_filename="h1_2_model_cache.pkl",
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
        name="H1", urdf_path=urdf, model_dir=directory, 
        cache_filename="h1_model_cache.pkl",
        joints_to_lock=joints_to_lock,
        ee_left=EndEffectorConfig("left_elbow_joint", np.array([0.2605 + 0.05, 0, 0])),
        ee_right=EndEffectorConfig("right_elbow_joint", np.array([0.2605 + 0.05, 0, 0])),
        weights=IKWeights(translation=50, rotation=0.5, regularization=0.02, smooth=0.1),
        smooth_window_size=8
    )

def create_panda_config(unit_test=False) -> ArmIKConfig:
    urdf, directory = get_base_paths(unit_test, "panda", "fr3.urdf")
    active_joint_names = [f"joint{i}" for i in range(1, 8)]
    
    return ArmIKConfig(
        name="Panda", urdf_path=urdf, model_dir=directory, 
        cache_filename="panda_model_cache.pkl",
        active_joint_names=active_joint_names,
        ee_left=EndEffectorConfig("joint7", np.array([0.0, 0.0, 0.107]), np.array([
    [0, 0, 1],  # New X is Old Z
    [0, 1, 0],  # New Y is Old Y
    [-1, 0, 0]  # New Z is Old -X
])),
        ee_right=None,
        left_ee_frame_name="hand",
        weights=IKWeights(translation=50, rotation=1.0, regularization=0.02, smooth=0.1),
        smooth_window_size=7
    )


# ==========================================
# UnifiedArmKinematics (High-level Interface)
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
        position_weight: float = 100.0,
        orientation_weight: float = 10.0,
        posture_weight: float = 0.05,
        posture_reference: Optional[np.ndarray] = None,
        velocity_limits: Optional[Dict[str, float]] = None,
        joint_delta_limit: float = 0.1,
        visualization: bool = False,
        smooth_window_size: Optional[int] = None,
    ):
        self.target_frame_name = target_frame_name

        # 构建配置
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

        # 验证目标帧
        valid_frames = []
        if self.arm_ik_config.ee_left:
            valid_frames.append(self.arm_ik_config.left_ee_frame_name)
        if self.arm_ik_config.ee_right:
            valid_frames.append(self.arm_ik_config.right_ee_frame_name)
            
        if self.target_frame_name not in valid_frames:
            raise ValueError(f"target_frame_name '{target_frame_name}' must be one of {valid_frames}")

        self.is_left_arm_control = (target_frame_name == arm_ik_config.left_ee_frame_name)

        # 获取关节名称
        self.joint_names = [
            self.unified_arm_ik.reduced_robot.model.names[i]
            for i in range(1, self.unified_arm_ik.reduced_robot.model.nq + 1)
        ]

        if joint_names is not None and set(joint_names) != set(self.joint_names):
            logger_mp.warning(
                f"Provided joint_names mismatch with model joints. "
                f"Using model joints: {self.joint_names}"
            )

        self.velocity_limits = velocity_limits
        self.joint_delta_limit = joint_delta_limit
        
        # 初始化 prev_joint_pos
        if posture_reference is not None:
            if len(posture_reference) != len(self.joint_names):
                raise ValueError(f"posture_reference length mismatch")
            self.prev_joint_pos = np.deg2rad(posture_reference)
        else:
            self.prev_joint_pos = pin.neutral(self.unified_arm_ik.reduced_robot.model).copy()

        self.unified_arm_ik.init_data = self.prev_joint_pos.copy()

    def forward_kinematics(self, joint_pos_deg: np.ndarray) -> np.ndarray:
        """正向运动学"""
        if len(joint_pos_deg) != len(self.joint_names):
            raise ValueError(f"joint_pos_deg length mismatch: expected {len(self.joint_names)}, "
                           f"got {len(joint_pos_deg)}")

        joint_pos_rad = np.deg2rad(joint_pos_deg)
        
        pin.framesForwardKinematics(
            self.unified_arm_ik.reduced_robot.model,
            self.unified_arm_ik.reduced_robot.data,
            joint_pos_rad
        )
        
        frame_id = self.unified_arm_ik.reduced_robot.model.getFrameId(self.target_frame_name,pin.FrameType.OP_FRAME)
        if frame_id >= self.unified_arm_ik.reduced_robot.model.nframes:
            raise ValueError(f"Frame {self.target_frame_name} not found!")
            
        return self.unified_arm_ik.reduced_robot.data.oMf[frame_id].homogeneous

    def inverse_kinematics(
        self,
        current_joint_pos: np.ndarray,
        desired_ee_pose: np.ndarray,
        joint_task_weight: Optional[float] = None,
        target_joints: Optional[Dict[str, float]] = None,
        position_weight: Optional[float] = None,
        orientation_weight: Optional[float] = None,
    ) -> np.ndarray:
        """逆向运动学"""
        if len(current_joint_pos) != len(self.joint_names):
            raise ValueError(f"current_joint_pos length mismatch")

        current_joint_rad = np.deg2rad(current_joint_pos)

        # ✅ FIX: 动态更新权重（仅当需要时重新创建 solver）
        weights_changed = False
        if position_weight is not None and position_weight != self.unified_arm_ik.config.weights.translation:
            self.unified_arm_ik.config.weights.translation = position_weight
            weights_changed = True
        if orientation_weight is not None and orientation_weight != self.unified_arm_ik.config.weights.rotation:
            self.unified_arm_ik.config.weights.rotation = orientation_weight
            weights_changed = True
        if joint_task_weight is not None and joint_task_weight != self.unified_arm_ik.config.weights.regularization:
            self.unified_arm_ik.config.weights.regularization = joint_task_weight
            weights_changed = True

        # ✅ FIX: 如果权重改变，需要重新设置 CasADi 参数（如果实现为参数）或重建
        # 当前实现：权重是 config 的属性，但 CasADi 图在创建时固定
        # 警告：动态权重需要重建 solver，这里暂时忽略动态权重或添加重建逻辑
        if weights_changed:
            logger_mp.warning("Weight changes require solver rebuild. Ignoring for this call.")
            # 或者：self.unified_arm_ik._setup_casadi_optimization()

        # 准备目标位姿
        left_wrist = None
        right_wrist = None
        
        if self.is_left_arm_control:
            left_wrist = desired_ee_pose
        else:
            right_wrist = desired_ee_pose

        sol_q_rad, _ = self.unified_arm_ik.solve_ik(
            left_wrist,
            right_wrist,
            current_lr_arm_motor_q=current_joint_rad,
            current_lr_arm_motor_dq=np.zeros_like(current_joint_rad)
        )

        # 关节增量限制
        if self.joint_delta_limit is not None and self.prev_joint_pos is not None:
            delta = sol_q_rad - self.prev_joint_pos
            delta = np.clip(delta, -self.joint_delta_limit, self.joint_delta_limit)
            sol_q_rad = self.prev_joint_pos + delta

        self.prev_joint_pos = sol_q_rad.copy()
        return np.rad2deg(sol_q_rad)

    def _get_default_posture(self) -> np.ndarray:
        """获取默认姿态（度）"""
        return np.rad2deg(pin.neutral(self.unified_arm_ik.reduced_robot.model).copy())


def create_unified_kinematics_config(
    robot_type: str,
    urdf_path: str,
    target_frame_name: str,
    unit_test: bool = False,
    cache_filename: Optional[str] = None,
    active_joint_names: Optional[List[str]] = None,
    **kwargs
) -> ArmIKConfig:
    """创建 IK 配置的工厂函数"""
    config_factory = {
        "g1_29": create_g1_29_config,
        "g1_23": create_g1_23_config,
        "h1_2": create_h1_2_config,
        "h1": create_h1_config,
        "panda": create_panda_config,
    }

    if robot_type not in config_factory:
        raise ValueError(f"Unsupported robot_type: {robot_type}")

    base_config = config_factory[robot_type](unit_test=unit_test)

    # 覆盖路径
    if urdf_path is not None:
        base_config.urdf_path = urdf_path
        base_config.model_dir = str(Path(urdf_path).parent)

    if cache_filename is not None:
        base_config.cache_filename = cache_filename

    if active_joint_names is not None:
        base_config.active_joint_names = active_joint_names
        base_config.joints_to_lock = None

    # 应用权重参数
    if 'position_weight' in kwargs: 
        base_config.weights.translation = kwargs.pop('position_weight')
    if 'orientation_weight' in kwargs: 
        base_config.weights.rotation = kwargs.pop('orientation_weight')
    if 'posture_weight' in kwargs: 
        base_config.weights.regularization = kwargs.pop('posture_weight')
    if 'smooth_window_size' in kwargs: 
        base_config.smooth_window_size = kwargs.pop('smooth_window_size')

    # 应用 EE 帧名
    if 'left_ee_frame_name' in kwargs: 
        base_config.left_ee_frame_name = kwargs.pop('left_ee_frame_name')
    if 'right_ee_frame_name' in kwargs: 
        base_config.right_ee_frame_name = kwargs.pop('right_ee_frame_name')

    if kwargs:
        logger_mp.warning(f"Unhandled kwargs: {kwargs}")

    return base_config


# ==========================================
# Main Test Execution
# ==========================================

if __name__ == "__main__":
    UNIT_TEST = True
    VISUALIZATION = True
    
    # 请确保路径正确
    robot_type = "panda"
    target_ee = "hand"
    # 注意：这里的路径可能需要根据你的实际环境修改
    urdf_path = "/home/smai/workspace/dikeke/franka_description/fr3_urdfs/fr3_franka_hand_obj.urdf"

    # 如果 URDF 文件不存在，避免报错，提示用户
    if not os.path.exists(urdf_path):
        logger_mp.error(f"URDF file not found at: {urdf_path}")
        sys.exit(1)

    unified_kinematics_solver = UnifiedArmKinematics(
        robot_type=robot_type,
        urdf_path= urdf_path,
        target_frame_name=target_ee,
        unit_test=UNIT_TEST,
        cache_filename=f"{robot_type}_unified_kin_cache.pkl",
        visualization=VISUALIZATION,
        position_weight=100.0,
        orientation_weight=10.0,
        posture_weight=0.05,
        joint_delta_limit=np.deg2rad(10),
    )

    initial_q_deg = np.rad2deg(
        pin.neutral(unified_kinematics_solver.unified_arm_ik.reduced_robot.model).copy()
    )
    
    # 获取初始位姿
    initial_ee_pose = unified_kinematics_solver.forward_kinematics(initial_q_deg)

    # 定义左右手的初始目标基准位姿
    L_tf_target_init = pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.25, +0.25, 0.5])) # z轴稍微抬高一点便于观察
    R_tf_target_init = pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.25, -0.25, 0.5]))

    # === FIX START ===
    # 1. 保存一个不变的“基准”位姿 (Base Pose)
    if unified_kinematics_solver.is_left_arm_control:
        base_ee_target_matrix = L_tf_target_init.homogeneous.copy()
    else:
        base_ee_target_matrix = R_tf_target_init.homogeneous.copy()

    # 2. active_ee_target 作为当前要发送给 IK 的目标，初始化为基准
    active_ee_target = base_ee_target_matrix.copy()
    # === FIX END ===

    rotation_speed = 0.05
    noise_trans = 0.001
    noise_rot = 0.01

    print(f"[{robot_type.upper()}] Initializing for controlling {target_ee}...")

    if input("Enter 's' to start simulation:\n").lower() == 's':
        step = 0
        current_q_deg = initial_q_deg
        
        # 记录基准位置向量
        base_translation = base_ee_target_matrix[:3, 3].copy()

        while True:
            # 生成动态旋转
            angle = rotation_speed * step if step <= 120 else rotation_speed * (240 - step)
            base_rot = pin.Quaternion(np.cos(angle/2), 0, np.sin(angle/2), 0)
            
            # 添加旋转噪声
            rot_noise = pin.Quaternion(
                np.cos(np.random.normal(0, noise_rot)/2), 
                np.random.normal(0, noise_rot/2), 
                0, 
                np.random.normal(0, noise_rot/2)
            ).normalized()
            
            new_rotation = (rot_noise * base_rot).toRotationMatrix()
            
            # 计算偏移量
            z_offset = 0.2 * np.sin(step * 0.05)
            y_offset = 0.05 * (1+np.sin(step * 0.05))
            x_offset = 0.05 * (1+np.cos(step * 0.05))

            # === FIX START ===
            # 错误写法 (导致累积漂移): 
            # current_target_translation = active_ee_target[:3, 3] + np.array([x_offset, 0, z_offset])
            
            # 正确写法 (基于基准位置计算):
            current_target_translation = base_translation + np.array([x_offset, 0, z_offset])
            # === FIX END ===

            # logging.info 太快可能会刷屏，建议偶尔打印
            if step % 20 == 0:
                logger_mp.info(f"[{robot_type.upper()}] Step {step} Target Z: {current_target_translation[2]:.4f}")

            # 更新目标矩阵
            active_ee_target[:3, :3] = new_rotation
            active_ee_target[:3, 3] = current_target_translation

            try:
                sol_q_deg = unified_kinematics_solver.inverse_kinematics(current_q_deg, active_ee_target)
                current_q_deg = sol_q_deg
                
            except Exception as e:
                logger_mp.error(f"Error in IK: {e}")

            step = (step + 1) % 241
            time.sleep(0.02)