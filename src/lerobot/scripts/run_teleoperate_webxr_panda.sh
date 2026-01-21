#!/bin/bash
# Run script for WebXR teleoperation of SimRobotPandaHil
#
# Usage:
#   bash scripts/run_teleoperate_webxr_panda.sh --config_path=configs/teleoperate_webxr_panda.yaml
#   bash scripts/run_teleoperate_webxr_panda.sh --config_path=path/to/custom_config.yaml --teleop_time_s=60

# Parse command line arguments
CONFIG_PATH=""
ADDITIONAL_ARGS=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --config_path=*)
            CONFIG_PATH="${1#*=}"
            shift
            ;;
        *)
            ADDITIONAL_ARGS="$ADDITIONAL_ARGS $1"
            shift
            ;;
    esac
done

# Check if config path is provided
if [ -z "$CONFIG_PATH" ]; then
    echo "Error: --config_path is required"
    echo "Usage: bash scripts/run_teleoperate_webxr_panda.sh --config_path=path/to/config.yaml [additional_args]"
    echo ""
    echo "Example:"
    echo "  bash scripts/run_teleoperate_webxr_panda.sh --config_path=configs/teleoperate_webxr_panda.yaml"
    echo "  bash scripts/run_teleoperate_webxr_panda.sh --config_path=configs/teleoperate_webxr_panda.yaml --teleop_time_s=60 --fps=20"
    exit 1
fi

# Check if config file exists
if [ ! -f "$CONFIG_PATH" ]; then
    echo "Error: Config file not found: $CONFIG_PATH"
    exit 1
fi

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=========================================="
echo "WebXR Teleoperation for SimRobotPandaHil"
echo "=========================================="
echo "Config: $CONFIG_PATH"
echo "Additional args: $ADDITIONAL_ARGS"
echo ""

# Run the teleoperation script
python -m lerobot.scripts.teleoperate_webxr_panda \
    --config_path="$CONFIG_PATH" \
    $ADDITIONAL_ARGS

echo ""
echo "Teleoperation session ended."
