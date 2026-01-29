# LeRobot Development Guide

## Project Overview

This is a customized fork of [huggingface/lerobot](https://github.com/huggingface/lerobot) for robot teleoperation with WebXR support and Human-in-the-Loop Reinforcement Learning (HIL-SERL) integration.

**Current Branch:** `hil-webxr-rot-cc` - WebXR rotation control with continuous training

### Key Customizations

- **WebXR Teleoperation**: Control robots via WebXR/VR through Zenoh messaging
- **HIL-SERL Integration**: Human-in-the-Loop reinforcement learning for real-time training
- **ROS2 Integration**: Interface with real robots via ROS2 controllers
- **Supre Robot Support**: Custom robot platform (双臂机器人系统)

## Project Structure

```
lerobot/
├── lerobot/              # Main package (symlinked from src/)
├── src/lerobot/          # Source code
│   ├── cameras/          # Camera drivers and calibration
│   ├── configs/          # Configuration files
│   │   ├── supre_robot_hil/    # HIL configs
│   │   └── teleoperate_webxr*.yaml  # WebXR configs
│   ├── datasets/         # Dataset handling
│   ├── envs/             # Simulation environments
│   ├── model/            # Neural network models
│   ├── policies/         # RL policies (ACT, Diffusion, etc.)
│   ├── robots/           # Robot implementations
│   │   ├── sim_robot_panda/     # Panda simulation with WebXR
│   │   ├── supre_robot_follower/ # Supre robot follower
│   │   └── ros2_follower/       # ROS2 interface
│   ├── scripts/          # Main scripts
│   │   ├── rl/           # RL training scripts
│   │   └── teleoperate_webxr_panda.py
│   ├── teleoperators/    # Teleoperation interfaces
│   │   ├── webxr/        # WebXR teleoperator
│   │   ├── supre_robot_leader/
│   │   └── ros2_leader/
│   └── utils/            # Utilities
├── ros2_ws/              # ROS2 workspace
├── run*.sh               # Quick start scripts
└── start_robot.sh        # Main launch script
```

## Development Workflow

### Environment Setup

```bash
# Setup environment without ROS2
./setup_no_ros2_env.sh

# Or setup with ROS2
./setup_binding_env.sh
```

### Running the Robot

```bash
# One-click start (teleop + autonomous)
./start_robot.sh

# Or manual steps:
# 1. Start ROS2 controller (in supre_robot_control project)
cd ~/workspace/supre_robot_control
./start_common_gripper_leader_follower.sh

# 2. Run lerobot teleoperation
cd ~/workspace/gitprj/lerobot/lerobot
./run_teleop.sh
```

### WebXR Teleoperation

```bash
# Teleoperate Panda robot via WebXR
python -m lerobot.scripts.teleoperate_webxr_panda

# Or using the script
./src/lerobot/scripts/run_teleoperate_webxr_panda.sh
```

### HIL-SERL Training

Configuration files in `src/lerobot/configs/supre_robot_hil/`:
- `env_config_supre_robot_follower_hil.yaml` - Real robot HIL config
- `env_config_supre_robot_follower_hil_sim_webxr.yaml` - Simulation with WebXR
- `train_config_hilserl_supre_robot.yaml` - Training configuration

### Robot Alignment

Before teleoperation, align the leader and follower arms:

```bash
source ./set_env.sh
python ./src/utils/teleop_controller_switcher.py
```

## Key Files Reference

| File | Purpose |
|------|---------|
| `start_robot.sh` | Main launch script for robot operation |
| `run_teleop.sh` | Teleoperation script |
| `teleop_aligner.py` | Leader-follower alignment |
| `teleop_controller_switcher.py` | Controller switching utility |
| `src/lerobot/teleoperators/webxr/` | WebXR teleoperation implementation |
| `src/lerobot/scripts/teleoperate_webxr_panda.py` | WebXR Panda control script |

## External Dependencies

- **ROS2 Controller Project**: `~/workspace/supre_robot_control`
- **Motor Driver**: `~/workspace/eu_motor` (意优电机驱动)
- **Gripper Project**: `~/workspace/misumi_gripper` (夹爪项目)

## Available Branches

- `main` - Upstream lerobot
- `hil` - Human-in-the-Loop features
- `hil-webxr` - HIL with WebXR support
- `hil-webxr-rot` - WebXR with rotation control
- `hil-webxr-rot-cc` - Current branch - continuous training

## Common Issues

### ROS2 Controller Issues
- Ensure ROS2 environment is sourced: `source /opt/ros/humble/setup.bash`
- Check controller status with `ros2 control list_controllers`

### WebXR Connection
- Zenoh is used for WebXR communication
- Check Zenoh connection key in configs

### Camera Issues
- Run `python -m lerobot.find_cameras` to detect cameras
- Calibrate using `python -m lerobot.calibrate`

## Testing

```bash
# Test WebXR both modes
python test_webxr_both_mode.py
```

## References

- [LeRobot Documentation](https://huggingface.co/docs/lerobot)
- [HIL-SERL Paper](https://hil-serl.github.io/)
- [WebXR Specification](https://immersive-web.github.io/webxr/)
