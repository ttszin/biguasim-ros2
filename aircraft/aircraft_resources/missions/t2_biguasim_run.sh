#!/bin/bash
# T2 Biguasim mission launcher — sets required env vars and runs the mission node.
# Usage: bash t2_biguasim_run.sh [conops_path]

export DRONE_TYPE=quad
export DRONE_ID=0

source /opt/ros/jazzy/setup.bash
source /home/teteu/aerial-autonomy-stack/aircraft/aircraft_ws/install/setup.bash

CONOPS="${1:-$(realpath "$(dirname "$0")/t2_biguasim_mission.yaml")}"

exec ros2 run mission mission --conops "$CONOPS"
