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

# --- 2. 核心黑魔法：禁用物理，强制设置位置 ---
def override_simulator_step_no_physics(simulator_instance):
    """
    替换 simulator 的 step 方法。
    使用 resetJointState 代替 setJointMotorControl2。
    效果：无视重力、无视阻力，绝对位置控制。
    """
    def no_physics_step(action: np.ndarray):
        for joint_name, idx in simulator_instance.flexible_joint.items():
            ia = simulator_instance.joint2idx[joint_name]
            
            # 原始 Action 处理（保持方向乘法）
            target_pos_rad = action[ia] * simulator_instance.joint_direction[ia]
            
            # 简单的限位保护 (防止超出 URDF 定义的极限报错)
            low = simulator_instance.action_low[ia]
            high = simulator_instance.action_high[ia]
            target_pos_rad = max(low, min(target_pos_rad, high))

            # 强制设置位置 (Teleport)
            p.resetJointState(simulator_instance.robot_id, idx, targetValue=target_pos_rad)
        
        # 必须调用这个来更新画面和碰撞体位置
        p.performCollisionDetection() 
        return simulator_instance.get_observation()

    simulator_instance.step = no_physics_step
    print(">>> [模式] 已启用无物理运动学模式")

# --- 3. 辅助函数：在屏幕上显示文字 ---
def draw_debug_text(text, life_time=0.5):
    # 在机器人头顶位置显示文字
    p.addUserDebugText(
        text=text,
        textPosition=[0, 0, 0.8],
        textColorRGB=[1, 0, 0], # 红色
        textSize=2.0,
        lifeTime=life_time
    )

# --- 4. 主逻辑 ---
def main():
    cfg = TestConfig()
    
    try:
        print(f"正在加载模型: {cfg.urdf_path}")
        robot = SimRobotHil(cfg)
        robot.connect()
    except Exception as e:
        print(f"错误: 无法加载机器人，请检查路径。\n{e}")
        return

    # 调整相机
    p.resetDebugVisualizerCamera(1.2, 0, -20, [0, 0, 0.5])
    
    # 启用无物理模式
    override_simulator_step_no_physics(robot.simulator)
    p.setGravity(0, 0, 0)

    joint_names = robot.get_joint_names()
    print(f"\n检测到关节列表: {joint_names}\n")

    # 初始化所有关节为 0
    current_positions = {name: 0.0 for name in joint_names}
    robot.write_goal_position(current_positions)
    time.sleep(1)

    try:
        # === 阶段 1: 逐个关节独立测试 ===
        print("=== 开始单关节测试 (每个关节转动 +/- 45度) ===")
        
        for idx, active_joint in enumerate(joint_names):
            print(f"--> 正在测试第 {idx+1} 个关节: {active_joint}")
            
            # 生成测试轨迹：0 -> 45 -> -45 -> 0 (总共涵盖 90 度范围)
            # 使用 linspace 生成平滑轨迹点
            traj_p1 = np.linspace(0, 45, 30)   # 0 到 45
            traj_p2 = np.linspace(45, -45, 60) # 45 到 -45
            traj_p3 = np.linspace(-45, 0, 30)  # -45 到 0
            full_traj = np.concatenate([traj_p1, traj_p2, traj_p3])

            for angle in full_traj:
                # 只有当前测试的关节动，其他保持 0
                target_pos = current_positions.copy() # 全 0
                target_pos[active_joint] = float(angle)
                
                robot.write_goal_position(target_pos)
                
                # 屏幕显示
                draw_debug_text(f"{active_joint}: {angle:.1f} deg", life_time=0.05)
                time.sleep(0.01) # 控制动画速度

            # 复位休息一下
            robot.write_goal_position(current_positions)
            time.sleep(0.5)

        # === 阶段 2: 组合运动测试 ===
        print("\n=== 单关节测试结束，开始组合演示 ===")
        draw_debug_text("COMBINED TEST", life_time=2)
        start_time = time.time()
        while True:
            t = time.time() - start_time
            
            pos_dict = {}
            for i, name in enumerate(joint_names):
                # 简单的正弦波，不同关节有相位差
                # 夹爪通常行程短，给小一点幅度
                if "gripper" in name:
                    angle = 45 + 45 * math.sin(t * 2) # 0~90
                else:
                    angle = 30 * math.sin(t * 1.5 + i * 0.5) # -30~30
                
                pos_dict[name] = angle
            
            robot.write_goal_position(pos_dict)
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n测试已停止。")
    finally:
        robot.disconnect()

if __name__ == "__main__":
    main()