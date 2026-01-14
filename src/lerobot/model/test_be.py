import matplotlib.pyplot as plt
import numpy as np
from .be_kinematics import BeRobotKinematics

PANDA_URDF_JOINT_NAMES = [
    "joint1", "joint2", "joint3", 
    "joint4", "joint5", "joint6", "joint7"
]
def test_kinematics_smoothness():
    # 模拟初始化 (假设有一个简单的 URDF)
    # 这里需要替换为你本地真实的 URDF 路径
    urdf_path = "/home/smai/workspace/dikeke/franka_description/fr3_urdfs/fr3_franka_hand_obj.urdf" 
    try:
        ik = BeRobotKinematics(urdf_path,
                               target_frame_name="hand",
                               joint_names=PANDA_URDF_JOINT_NAMES)
    except Exception as e:
        print(f"请确保 URDF 路径正确: {e}")
        return

    # 初始状态
    current_q = np.zeros(len(ik.joint_names)) 
    
    # 1. 模拟目标轨迹：一个圆周运动 + 一个突然的阶跃跳变
    steps = 100
    times = np.linspace(0, 2*np.pi, steps)
    target_positions = []
    
    for i, t in enumerate(times):
        x = 0.4 + 0.1 * np.cos(t)
        y = 0.1 * np.sin(t)
        z = 0.3
        # 在第 50 步时，目标点突然跳变 20cm
        if i > 50:
            z += 0.2 
        
        T = np.eye(4)
        T[:3, 3] = [x, y, z]
        target_positions.append(T)

    # 2. 执行 IK 追踪
    history_q = []
    actual_pos = []
    
    print("开始追踪测试...")
    for i in range(steps):
        current_q = ik.inverse_kinematics(current_q, target_positions[i])
        history_q.append(current_q.copy())
        
        # 记录实际达到的末端位置
        res_T = ik.robot.get_T_world_frame(ik.target_frame_name)
        actual_pos.append(res_T[:3, 3].copy())

    history_q = np.array(history_q)
    actual_pos = np.array(actual_pos)
    target_pos_arr = np.array([T[:3, 3] for T in target_positions])

    # 3. 可视化
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

    # 子图 1: 关节空间（展示平滑度）
    for j in range(len(ik.joint_names)):
        ax1.plot(history_q[:, j], label=f"Joint {j}")
    ax1.set_title("Joint Trajectory (Smoothness Check)")
    ax1.set_ylabel("Degrees")
    ax1.axvline(x=50, color='r', linestyle='--', alpha=0.5, label='Target Jump')
    ax1.legend(loc='upper right', fontsize='small', ncol=2)
    ax1.grid(True)

    # 子图 2: 末端执行器 Z 轴（展示追踪能力与阶跃响应）
    ax2.plot(target_pos_arr[:, 2], 'r--', label="Target Z", alpha=0.6)
    ax2.plot(actual_pos[:, 2], 'b-', label="Actual Z")
    ax2.set_title("End-Effector Z Tracking")
    ax2.set_ylabel("Meters")
    ax2.set_xlabel("Time Step")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.show()

    # 4. 打印分析
    max_delta = np.max(np.abs(np.diff(history_q, axis=0)))
    print(f"最大关节跳变步长: {max_delta:.4f} 度")
    if max_delta < 10: # 如果跳变远小于直接解 IK 的结果
        print("结论：跳变成功受到约束，运动轨迹平滑。")
    else:
        print("结论：仍存在较大跳变，请调小 joint_delta_limit。")

if __name__ == "__main__":
    test_kinematics_smoothness()