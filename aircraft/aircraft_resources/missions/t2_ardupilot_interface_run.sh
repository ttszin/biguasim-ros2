#!/bin/bash
# Launches ardupilot_interface under the /Drone0 namespace required by the reposition service.
# Usage: bash t2_ardupilot_interface_run.sh

source /opt/ros/jazzy/setup.bash
source /tmp/px4_ws/install/setup.bash
source /tmp/ground_msgs_ws/install/setup.bash
source /home/teteu/aerial-autonomy-stack/aircraft/aircraft_ws/install/setup.bash

exec ros2 run autopilot_interface ardupilot_interface --ros-args -r __ns:=/Drone0
