#!/bin/bash
# T2 Direct Runner — BiguaSim cmd_pos_yaw only, no ArduPilot, no MAVROS.
#
# Usage:
#   bash t2_direct_run.sh [--viewport] [--calibrate] [extra args...]
#
# Examples:
#   bash t2_direct_run.sh --viewport
#   bash t2_direct_run.sh --viewport --calibrate --water-x 25 --water-y 0
#   bash t2_direct_run.sh --viewport --water-x 25 --descent-z -2.0

source /opt/ros/jazzy/setup.bash

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

python3 "$SCRIPT_DIR/../t2_direct_runner.py" "$@"
