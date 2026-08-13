#!/bin/bash
# Starts Rover SITL for the BlueBoat, standalone (JSON interface on port 9002,
# MAVLink on TCP 5760 — same ports as t2_sitl_run.sh's ArduCopter instance,
# since these are meant to run one at a time, never both together).
# Deletes eeprom.bin first so --defaults params are always respected.
# Usage: bash t2_sitl_run_blueboat.sh

EEPROM="/home/teteu/ardupilot/Tools/autotest/eeprom.bin"
PARAMS="$(realpath "$(dirname "$0")/t2_biguasim_blueboat.parm")"
SIM_VEHICLE="/home/teteu/ardupilot/Tools/autotest/sim_vehicle.py"

if [ -f "$EEPROM" ]; then
    echo "[t2_sitl_blueboat] Deleting eeprom.bin to ensure clean params..."
    rm -f "$EEPROM"
fi

echo "[t2_sitl_blueboat] Starting Rover SITL..."
cd /home/teteu/ardupilot/Tools/autotest

# No -A "--rate ..." override here, unlike t2_sitl_run.sh's ArduCopter instance.
# Confirmed in ardupilot/libraries/AP_Scheduler/AP_Scheduler.cpp:44-47:
# SCHEDULER_DEFAULT_LOOP_RATE is conditionally 400 for Copter/Heli/ArduSub, but
# 50 for everything else — which includes Rover. BiguaSim's real delivery rate
# (measured live for the DjiMatrice case, ~120-135Hz) is already comfortably
# above Rover's 50Hz default expectation, so the loop-rate mismatch that broke
# DjiMatrice arming shouldn't apply here. If live testing (see README) shows
# otherwise, add -A "--rate ..." + a matching SCHED_LOOP_RATE here, same
# reasoning as t2_sitl_run.sh.
python3 "$SIM_VEHICLE" \
    -v Rover \
    --model JSON \
    --add-param-file="$PARAMS" \
    --no-mavproxy \
    -I0
