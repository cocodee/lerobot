#!/bin/bash
set -e

# --- run_server.sh ---
# 启动 LeRobot Policy Inference Action Server

# 1. Source ROS 2 环境
source /opt/ros/humble/setup.bash
source /lerobot/lerobot/ros2_ws/install/setup.bash

# 2. 激活 Python 虚拟环境（可选）
VENV_ACTIVATE="/opt/venv/bin/activate"
if [ -f "$VENV_ACTIVATE" ]; then
    source "$VENV_ACTIVATE"
fi

export PYTHONPATH="/opt/ros/humble/lib/python3.10/site-packages:/opt/venv/lib/python3.10/site-packages:/lerobot/lerobot/src:$PYTHONPATH"

echo "PYTHONPATH:" $PYTHONPATH
# 3. 切换到工作空间根目录，保证 ros2 launch 能找到包
cd /lerobot/lerobot/ros2_ws

# 4. 启动 launch 文件
echo "Launching the Policy Inference Server..."
ros2 launch lerobot_ros2 inference.launch.py
