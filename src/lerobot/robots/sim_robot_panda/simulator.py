import mujoco
import mujoco.viewer
import numpy as np
import time
import logging
import os
from typing import Dict, Tuple, Optional
from ..sim_robot.config_sim_robot import SimRobotPandaHilConfig
import xml.etree.ElementTree as ET

logger = logging.getLogger(__name__)

class MujocoSimulator:
    def __init__(self, config: SimRobotPandaHilConfig, headless: bool = False):
        self.config = config
        self.headless = headless
        
        # 调用上面写的合并函数
        try:
            robot_path = os.path.abspath(self.config.xml_path)
            # 这里调用上面定义的逻辑
            self.model = self._build_model_with_scene(robot_path)
            logger.info("Successfully merged robot into scene at 1.5m height")
        except Exception as e:
            logger.error(f"Failed to load merged model: {e}")
            # 回退...
            self.model = mujoco.MjModel.from_xml_path(robot_path)


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

    def _build_model_with_scene(self, robot_xml_path):
        """
        通过 Python 代码合并场景和机器人模型，并返回 MjModel 对象。
        """
        mount_height = 1.5
        robot_path = os.path.abspath(robot_xml_path)
        robot_dir = os.path.dirname(robot_path)
    
        # 1. 解析机器人原始 XML
        robot_tree = ET.parse(robot_xml_path)
        robot_root = robot_tree.getroot()
    
        # 2. 创建场景的基础结构 (String)
        scene_xml_base = f"""
        <mujoco model="scene_with_robot">
            <statistic extent="2" center="0 0 1"/>
            <option timestep="0.002"/>
            <visual>
                <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
                <rgba haze="0.15 0.25 0.35 1"/>
                <global azimuth="120" elevation="-20"/>
            </visual>
            <asset>
                <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
                <texture type="2d" name="groundplane" builtin="checker" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" mark="edge" markrgb="0.8 0.8 0.8" width="300" height="300"/>
                <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
            </asset>
            <worldbody>
                <light pos="0 0 3" dir="0 0 -1" directional="true"/>
                <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
                <camera name="side_view" pos="0 -2 1.5" xyaxes="1 0 0 0 0 1"/>
                
                <!-- 我们要把机械臂放进这个 mount 里 -->
                <body name="robot_mount" pos="0 0 {mount_height}">
                </body>
            </worldbody>
        </mujoco>
        """
        
        # 3. 将字符串转为 Element 对象
        scene_root = ET.fromstring(scene_xml_base)
        mount_body = scene_root.find(".//body[@name='robot_mount']")
        scene_asset = scene_root.find("asset")
    
        # 4. 提取机器人 XML 中的所有 Assets (Mesh, Material, Texture) 并合并到场景
        robot_assets = robot_root.find("asset")
        if robot_assets is not None:
            for asset in robot_assets:
                scene_asset.append(asset)
    
        # 5. 提取机器人 XML 中的 Worldbody 内容并放入 mount_body
        robot_worldbody = robot_root.find("worldbody")
        if robot_worldbody is not None:
            for element in robot_worldbody:
                mount_body.append(element)
    
        # 6. 处理编译选项 (如 mesh 路径)
        # 如果机器人 XML 有 compiler 标签，直接复制过来
        robot_compiler = robot_root.find("compiler")
        if robot_compiler is not None:
            scene_root.insert(0, robot_compiler)
        else:
            # 如果没有，手动添加一个确保能找到 mesh 路径
            compiler = ET.Element("compiler", meshdir=robot_dir, texturedir=robot_dir)
            scene_root.insert(0, compiler)
    
        # 7. 导出最终的 XML 字符串
        merged_xml_str = ET.tostring(scene_root, encoding='unicode')
    
        # 8. 加载模型
        # 注意：一定要传入 robot_dir 作为 assets 路径，否则找不到 .stl 文件
        return mujoco.MjModel.from_xml_string(merged_xml_str)
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

    def get_observation(self) -> Dict[str, float]:
        """
        返回主脚本期望的字典格式：{'joint1.pos': value, 'joint1.vel': value, ...}
        """
        obs = {}
        for i, name in enumerate(self.joint_names):
            # 获取位置和速度
            q_pos = self.data.qpos[self.joint_qpos_adr[i]]
            q_vel = self.data.qvel[self.joint_qvel_adr[i]]
            
            # 这里的 Key 必须与主脚本 self._joint_names 中的定义一致
            # 如果脚本找的是 'joint1.pos'，即使模型里叫 'panda_joint1'，这里也建议映射回 'joint1'
            # 或者确保主脚本的配置里关节名匹配
            logic_name = f"joint{i+1}" 
            obs[f"{logic_name}.pos"] = float(q_pos)
            obs[f"{logic_name}.vel"] = float(q_vel)
            
        return obs

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
            