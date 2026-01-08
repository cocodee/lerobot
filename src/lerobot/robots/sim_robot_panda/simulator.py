import mujoco
import mujoco.viewer
import numpy as np
import time
import logging
import os
from typing import Dict, Tuple, Optional
from ..sim_robot.config_sim_robot import SimRobotPandaHilConfig

logger = logging.getLogger(__name__)

class MujocoSimulator:
    def __init__(self, config: SimRobotPandaHilConfig, headless: bool = False):
        self.config = config
        self.headless = headless
        
        # --- 修改开始：构建环境并加载模型 ---
        
        # 1. 获取机械臂文件的绝对路径 (防止相对路径在字符串加载时出错)
        robot_path = os.path.abspath(self.config.xml_path)
        
        # 2. 动态生成包含环境和机械臂位置的 XML 字符串
        # 注意：如果你的模型是 .urdf 格式，MuJoCo 的 <include> 标签可能无法直接支持。
        # 建议先将 URDF 拖入 MuJoCo 模拟器并 Save xml 为 .xml (MJCF) 格式，效果最好。
        scene_xml = self._create_scene_xml(robot_path)

        # 3. 从生成的字符串加载模型
        try:
            # 使用 from_xml_string 加载构建好的场景
            # basedir 是为了让 MuJoCo 找到机械臂文件旁边的 mesh 资源
            self.model = mujoco.MjModel.from_xml_string(scene_xml, list(), os.path.dirname(robot_path))
        except Exception as e:
            logger.error(f"Failed to load MuJoCo model: {e}")
            # 如果加载失败（例如因为 URDF include 问题），回退到原始加载方式
            logger.warning("Falling back to loading robot file directly (No Environment)...")
            self.model = mujoco.MjModel.from_xml_path(self.config.urdf_path1)

        # --- 修改结束 ---

        self.data = mujoco.MjData(self.model)

        # 4. 初始化渲染器
        self.renderer = mujoco.Renderer(self.model, height=480, width=640)

        # 5. 初始化可视化窗口
        self.viewer = None
        if not self.headless:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        # 6. 建立名称到索引的映射
        self.joint_names = [
            "joint1", "joint2", "joint3", "joint4", 
            "joint5", "joint6", "joint7", 
        ]
        
        self.joint_ids = []
        self.joint_qpos_adr = []
        self.joint_qvel_adr = []

        for name in self.joint_names:
            j_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if j_id == -1:
                logger.warning(f"Joint {name} not found in MJCF model.")
            self.joint_ids.append(j_id)
            
            if j_id != -1:
                self.joint_qpos_adr.append(self.model.jnt_qposadr[j_id])
                self.joint_qvel_adr.append(self.model.jnt_dofadr[j_id])
            else:
                self.joint_qpos_adr.append(None)
                self.joint_qvel_adr.append(None)

        # 初始刷新一下，确保位置正确
        mujoco.mj_forward(self.model, self.data)
        if self.viewer:
            self.viewer.sync()

    def _create_scene_xml(self, robot_file_path: str) -> str:
        """
        创建一个包含地板、光照、背景以及固定在高处底座的机械臂的 XML 场景。
        """
        # 机械臂安装高度 (米)，模拟人肩膀高度
        mount_height = 1.5 
        
        xml_content = f"""
        <mujoco model="scene_with_robot">
            <statistic extent="2" center="0 0 1"/>
            <option timestep="0.002"/>

            <!-- 视觉设置：天空盒、雾气 -->
            <visual>
                <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
                <rgba haze="0.15 0.25 0.35 1"/>
                <global azimuth="120" elevation="-20"/>
            </visual>

            <!-- 资产：纹理和材质 -->
            <asset>
                <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
                <texture type="2d" name="groundplane" builtin="checker" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" mark="edge" markrgb="0.8 0.8 0.8" width="300" height="300"/>
                <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
            </asset>

            <worldbody>
                <!-- 环境对象：灯光和地板 -->
                <light pos="0 0 3" dir="0 0 -1" directional="true"/>
                <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>

                <!-- 额外的环境相机 (可选) -->
                <camera name="side_view" pos="0 -2 1.5" xyaxes="1 0 0 0 0 1"/>

                <!-- 机械臂底座：将机械臂固定在这个 Body 上 -->
                <!-- pos="0 0 {mount_height}" 将机械臂抬高 -->
                <body name="robot_mount" pos="0 0 {mount_height}" euler="0 0 0">
                    <!-- 引用外部机械臂文件 -->
                    <include file="{robot_file_path}"/>
                </body>
            </worldbody>
        </mujoco>
        """
        return xml_content

    def step(self, joint_positions: np.ndarray) -> np.ndarray:
        for i, target_pos in enumerate(joint_positions):
            if i < len(self.joint_names):
                qpos_adr = self.joint_qpos_adr[i]
                if qpos_adr is not None:
                    self.data.qpos[qpos_adr] = target_pos
                    self.data.qvel[self.joint_qvel_adr[i]] = 0.0

        mujoco.mj_forward(self.model, self.data)

        if self.viewer:
            self.viewer.sync()

        return self.get_observation()

    def get_observation(self) -> np.ndarray:
        pos, vel = self.get_joint_states()
        return np.concatenate([pos, vel])

    def get_joint_states(self) -> Tuple[np.ndarray, np.ndarray]:
        positions = []
        velocities = []

        for i, _ in enumerate(self.joint_names):
            qpos_adr = self.joint_qpos_adr[i]
            qvel_adr = self.joint_qvel_adr[i]

            if qpos_adr is not None:
                positions.append(self.data.qpos[qpos_adr])
                velocities.append(self.data.qvel[qvel_adr])
            else:
                positions.append(0.0)
                velocities.append(0.0)

        return np.array(positions), np.array(velocities)

    def get_camera_images(self) -> Dict[str, np.ndarray]:
        images = {}
        # 注意：现在场景中可能包含我们在 XML 里定义的 side_view 相机
        camera_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_CAMERA, i) 
            for i in range(self.model.ncam)
        ]

        for cam_name in camera_names:
            if not cam_name: continue
            
            self.renderer.update_scene(self.data, camera=cam_name)
            img = self.renderer.render()
            images[cam_name] = img
        
        return images

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        if self.viewer:
            self.viewer.sync()

    def close(self):
        if self.viewer:
            self.viewer.close()
            self.viewer = None