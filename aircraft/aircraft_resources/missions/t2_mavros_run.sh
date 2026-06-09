#!/bin/bash
# Launches MAVROS and requests all data streams once connected.
# Usage: bash t2_mavros_run.sh

source /opt/ros/jazzy/setup.bash

# --- ALTERAÇÃO AQUI: Adicionado o pipe com grep para filtrar o aviso chato ---
ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5760 2>&1 | grep --line-buffered -v "RTT too high" &
MAVROS_PID=$!

echo "Waiting for MAVROS to connect..."
until ros2 topic echo /mavros/state --once 2>/dev/null | grep -q "connected: true"; do
    sleep 1
done

echo "MAVROS connected. Requesting data streams..."
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate \
    "{stream_id: 0, message_rate: 10, on_off: true}"
echo "Streams requested."

wait $MAVROS_PID