#!/usr/bin/env python3
"""
WebXR 差分控制示例脚本

使用 Differential IK 替代传统的 Absolute IK
实现稳定的控制，避免 wrist roll 抖动
"""

import argparse
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R
import sys
import os

# 添加路径
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from lerobot.robots.sim_robot_panda.be_robot_kinematics import BeRobotKinematics
from lerobot.teleoperators.webxr.webxr_differential_wrapper import WebXRDifferentialWrapper
from lerobot.teleoperators.webxr.teleop_webxr import WebxrControlWrapper


def load_config(config_path):
    """加载配置文件"""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def main():
    parser = argparse.ArgumentParser(description="WebXR 差分控制 - 稳定的机械臂遥操")
    parser.add_argument('--config', type=str, required=True,
                        help='配置文件路径')
    parser.add_argument('--dt', type=float, default=0.02,
                        help='时间步长 (秒)')
    parser.add_argument('--damping', type=float, default=1e-4,
                        help='Jacobian 阻尼系数')
    parser.add_argument('--wrist_gain', type=float, default=2.0,
                        help='wrist 关节稳定增益')
    parser.add_argument('--posture_gain', type=float, default=0.2,
                        help='姿态正则化增益')
    parser.add_argument('--max_delta_pos', type=float, default=0.01,
                        help='最大位置变化量 (米)')
    parser.add_argument('--max_delta_rot', type=float, default=0.1,
                        help='最大旋转变化量 (弧度)')

    args = parser.parse_args()

    # 加载配置
    config = load_config(args.config)

    print("=== WebXR 差分控制初始化 ===")
    print(f"配置文件: {args.config}")
    print(f"时间步长: {args.dt}s")
    print(f"阻尼系数: {args.damping}")
    print(f"Wrist 增益: {args.wrist_gain}")
    print(f"姿态增益: {args.posture_gain}")
    print(f"最大位置变化: {args.max_delta_pos}m")
    print(f"最大旋转变化: {args.max_delta_rot}rad")
    print()

    # 创建机器人运动学模型
    print("1. 创建机器人运动学模型...")
    kin = BeRobotKinematics(
        robot_type=config.get('robot_type', 'sim_robot_panda'),
        target_frame_name=config.get('target_frame_name', 'panda_hand'),
        posture_reference=config.get('reference_posture', [0.0]*9)
    )
    print("   ✓ 机器人运动学模型创建成功")

    # 创建差分控制包装器
    print("2. 创建差分控制包装器...")
    diff_wrapper = WebXRDifferentialWrapper(
        kin=kin,
        dt=args.dt,
        damping=args.damping,
        wrist_gain=args.wrist_gain,
        posture_gain=args.posture_gain,
        max_delta_pos=args.max_delta_pos,
        max_delta_rot=args.max_delta_rot,
        wrist_joint_name=config.get('wrist_joint_name', 'panda_joint7')
    )
    print("   ✓ 差分控制包装器创建成功")

    # 创建 WebXR 控制包装器
    print("3. 创建 WebXR 控制包装器...")
    teleop_wrapper = WebxrControlWrapper(
        config=config,
        control_callback=diff_wrapper.update
    )
    print("   ✓ WebXR 控制包装器创建成功")

    # 主循环
    print("\n=== 开始 WebXR 差分控制 ===")
    print("WebXR 模式说明:")
    print("- TRANSLATE: 仅控制位置")
    print("- ROTATE: 仅控制姿态")
    print("- BOTH: 同时控制位置和姿态")
    print("- IDLE: 停止控制")
    print("\n控制特性:")
    print("- ✓ 使用差分 IK，避免绝对 IK 的跳变问题")
    print("- ✓ Wrist roll 通过 nullspace 稳定")
    print("- ✓ 数值稳定，不会抖动")
    print("- ✓ 限幅保护，确保安全性")
    print()

    try:
        # 运行控制循环
        teleop_wrapper.run()
    except KeyboardInterrupt:
        print("\n=== 控制停止 ===")
        print("用户中断，安全退出")
    except Exception as e:
        print(f"\n=== 错误 ===")
        print(f"控制过程中发生错误: {e}")
        print("详细错误信息:")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()