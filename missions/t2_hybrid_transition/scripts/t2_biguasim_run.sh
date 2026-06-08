#!/bin/bash
# T2 mission node launcher — sets required env vars and runs the mission node.
#
# Usage: bash t2_biguasim_run.sh [path/to/conops.yaml]
#        Default conops: ../t2_biguasim_mission.yaml (relative to this script)
#
# Requires aerial-autonomy-stack built. Set STACK_WS env var if not at ~/aerial-autonomy-stack.

STACK_WS="${STACK_WS:-$HOME/aerial-autonomy-stack/aircraft/aircraft_ws}"

export DRONE_TYPE=quad
export DRONE_ID=0

source /opt/ros/jazzy/setup.bash
source "$STACK_WS/install/setup.bash"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONOPS="${1:-$SCRIPT_DIR/../t2_biguasim_mission.yaml}"

exec ros2 run mission mission --conops "$CONOPS"
