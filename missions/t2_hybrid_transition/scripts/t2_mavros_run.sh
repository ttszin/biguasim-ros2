#!/bin/bash
# Launches MAVROS and requests all data streams once connected.
#
# Usage: bash t2_mavros_run.sh [fcu_url]
#        Default fcu_url: tcp://127.0.0.1:5760

source /opt/ros/jazzy/setup.bash

FCU_URL="${1:-tcp://127.0.0.1:5760}"

ros2 launch mavros apm.launch fcu_url:=$FCU_URL 2>&1 | grep --line-buffered -v "RTT too high" &
MAVROS_PID=$!

echo "Waiting for MAVROS to connect to $FCU_URL ..."
until ros2 topic echo /mavros/state --once 2>/dev/null | grep -q "connected: true"; do
    sleep 1
done

echo "MAVROS connected. Requesting data streams..."
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate \
    "{stream_id: 0, message_rate: 10, on_off: true}"
echo "Streams requested."

wait $MAVROS_PID