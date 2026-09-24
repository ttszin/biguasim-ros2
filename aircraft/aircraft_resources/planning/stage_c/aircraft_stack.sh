#!/bin/bash
# Runs INSIDE the aircraft-image container (ROS 2 Humble, network host): the ROS side of the stack for one flight.
#   MAVROS -> ardupilot_interface -> mission (the T2 t2_aircraft.yml.erb, minus tmux and the ROV bridge).
# usage: aircraft_stack.sh <mission.yaml> <log_dir>
# The mission node shuts itself down when the mission ends, so this script (and the container) exit then.
MISSION=$1
LOGS=${2:-/results}
set +e
source /opt/ros/humble/setup.bash
source /aas/aircraft_ws/install/setup.bash
export DRONE_TYPE=${DRONE_TYPE:-quad} DRONE_ID=${DRONE_ID:-0} AUTOPILOT=${AUTOPILOT:-ardupilot}

ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5760@ > $LOGS/mavros.log 2>&1 &
echo "[stack] waiting for MAVROS to connect to the SITL"
for i in $(seq 1 240); do
  ros2 topic echo /mavros/state --once 2>/dev/null | grep -q "connected: true" && break
  sleep 1
done
echo "[stack] MAVROS connected."
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 10, on_off: true}" > /dev/null 2>&1
ros2 param set /mavros/sys conn_timeout 60.0 > /dev/null 2>&1

ros2 run autopilot_interface ardupilot_interface --ros-args -r __ns:=/Drone${DRONE_ID} > $LOGS/interface.log 2>&1 &
for i in $(seq 1 240); do
  ros2 topic echo /mavros/state --once 2>/dev/null | grep -qE "system_status: (3|4)" && break
  sleep 1
done
echo "[stack] FCU ready (system_status 3/4). Starting the mission: $MISSION"
ros2 run mission mission --conops $MISSION --ros-args -r __ns:=/Drone${DRONE_ID} 2>&1 | tee $LOGS/mission.log
echo "[stack] mission process ended"
kill %1 %2 2>/dev/null
