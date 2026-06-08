#!/bin/bash
# Launches ardupilot_interface under the /Drone0 namespace required by the reposition service.
#
# Usage: bash t2_ardupilot_interface_run.sh
#
# Requires aerial-autonomy-stack built. Set STACK_WS env var if not at ~/aerial-autonomy-stack.

STACK_WS="${STACK_WS:-$HOME/aerial-autonomy-stack/aircraft/aircraft_ws}"

source /opt/ros/jazzy/setup.bash
source "$STACK_WS/install/setup.bash"

exec ros2 run autopilot_interface ardupilot_interface --ros-args -r __ns:=/Drone0
