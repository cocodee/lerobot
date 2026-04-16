set -e

source ~/miniconda3/etc/profile.d/conda.sh
source ./setup_binding_env.sh

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
cd "$SCRIPT_DIR"

CONFIG_FILE_PATH="$SCRIPT_DIR/src/lerobot/teleoperators/supre_robot_leader/trunk_teleoperate.yaml"

PYTHONPATH=src python -m lerobot.repeat_joint_trajectory \
    --robot.type=supre_robot_follower \
    --robot.id=supre_follower \
    --cycles=10 \
    --trajectory_duration=6.0 \
    --settle_time_s=2.0 \
    --pause_for_measurement=true \
    --config_path="$CONFIG_FILE_PATH"    
