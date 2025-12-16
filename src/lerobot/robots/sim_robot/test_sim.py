import time
import math
import numpy as np
import pybullet as p
from dataclasses import dataclass, field
from typing import List, Dict

# 引入你的类 (假设都在正确路径下)
# 请根据实际文件结构调整 import 路径
from lerobot.robots.sim_robot import SimRobotHil
from lerobot.robots.sim_robot import SimRobotHilConfig

# --- 1. 配置部分 ---
@dataclass
class TestConfig(SimRobotHilConfig):
    # 这里填入你真实的 URDF 路径
    urdf_path1: str = "/home/smai/workspace/dc_dir/sim_lerobot/rf2502_new_3/urdf/rf2502_new_3.urdf" 
    # 确保这里的名字和 URDF 里的对应
    target_frame_name: str = "left_hand_base" 
    
    # 定义测试用的相机（可选）
    cameras: Dict = field(default_factory=lambda: {})

def override_simulator_step_no_physics(simulator_instance):
    """
    [核心黑魔法]
    动态修改 simulator 实例的 step 方法。
    将原本的电机控制(setJointMotorControl2) 替换为 强制位置重置(resetJointState)。
    这样可以完全消除重力、惯性和阻力的影响，实现“指哪打哪”的可视化。
    """
    def no_physics_step(action: np.ndarray):
        # 遍历所有活动关节
        for joint_name, idx in simulator_instance.flexible_joint.items():
            ia = simulator_instance.joint2idx[joint_name]
            
            # 获取目标弧度 (保持原有的方向处理逻辑)
            target_pos = action[ia] * simulator_instance.joint_direction[ia]
            
            # 限位保护
            if target_pos < simulator_instance.action_low[ia]:
                target_pos = simulator_instance.action_low[ia]
            if target_pos > simulator_instance.action_high[ia]:
                target_pos = simulator_instance.action_high[ia]

            # 关键：使用 resetJointState 直接设置位置，绕过物理引擎
            p.resetJointState(simulator_instance.robot_id, idx, targetValue=target_pos)
        
        # 刷新渲染，不进行物理步进
        p.performCollisionDetection() 
        return simulator_instance.get_observation()

    # 将实例的方法替换掉
    simulator_instance.step = no_physics_step
    print(">>> 已启用无物理模式 (Direct Kinematics Mode)")

def main():
    # 1. 初始化配置
    # 注意：如果你的 SimRobotHilConfig 需要特定参数，请在这里补充
    cfg = TestConfig()
    
    print("正在初始化机器人...")
    try:
        robot = SimRobotHil(cfg)
        robot.connect()
    except Exception as e:
        print(f"初始化失败，请检查 URDF 路径或配置: {e}")
        return

    # 2. 调整 PyBullet 视角 (放大看清细节)
    p.resetDebugVisualizerCamera(
        cameraDistance=1.0, 
        cameraYaw=0, 
        cameraPitch=-20, 
        cameraTargetPosition=[0, 0, 0.5]
    )

    # 3. [关键] 替换 step 方法以去除物理影响
    # 这样即使你没有修改 simulator.py 的源码，运行此脚本时也是无重力的
    override_simulator_step_no_physics(robot.simulator)
    
    # 额外保险：将全局重力设为0
    p.setGravity(0, 0, 0)

    # 4. 获取关节名称列表
    joint_names = robot.get_joint_names()
    print(f"检测到的关节: {joint_names}")

    print("开始正弦波动作测试 (按 Ctrl+C 停止)...")
    
    start_time = time.time()
    
    try:
        while True:
            t = time.time() - start_time
            
            # 构造目标位置字典
            target_positions = {}
            
            for i, name in enumerate(joint_names):
                # 生成一个 -30度 到 +30度 的正弦波运动
                # 不同关节加上相位差(i * 0.5)，让动作看起来像波浪
                angle_deg = 30.0 * math.sin(2.0 * t + i * 0.5)
                
                # 如果是 gripper (通常范围较小)，特殊处理
                if "gripper" in name:
                    # 假设夹爪范围 0-100 或类似，这里简单设为正弦变化
                    angle_deg = 50 + 40 * math.sin(3.0 * t)
                
                target_positions[name] = angle_deg

            # 5. 写入目标位置
            # SimRobotHil 会将角度转换为弧度，并通过 step (已被我们要替换) 执行
            robot.write_goal_position(target_positions)
            
            # 控制循环频率，大约 50Hz
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n测试停止")
    finally:
        robot.disconnect()

if __name__ == "__main__":
    main()