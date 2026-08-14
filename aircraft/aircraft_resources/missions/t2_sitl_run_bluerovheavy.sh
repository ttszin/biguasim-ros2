#!/bin/bash
# Starts ArduSub SITL for the BlueROVHeavy, standalone (JSON interface on port
# 9002, MAVLink on TCP 5760 — same ports as t2_sitl_run.sh's ArduCopter
# instance, t2_sitl_run_blueboat.sh's Rover instance, and
# t2_sitl_run_bluerov2.sh's other ArduSub instance, since these are meant to
# run one at a time, never together).
#
# Purely diagnostic for now: used to test whether BiguaSim/Unreal's
# deterministic hang on BlueROV2 spawn (see biguasim_bridge/README.md,
# "Position-hold for BlueROV2 (ArduSub)") is specific to that vehicle's own
# asset, or a broader ArduSub/BiguaSim issue — BlueROVHeavy shares ArduSub
# and include_depth_sensor=True with BlueROV2, so if it does NOT hang, that
# points squarely at BlueROV2BP's own asset rather than anything Sub-related.
#
# Deletes eeprom.bin first so --defaults params are always respected.
# Usage: bash t2_sitl_run_bluerovheavy.sh

EEPROM="/home/teteu/ardupilot/Tools/autotest/eeprom.bin"
PARAMS="$(realpath "$(dirname "$0")/t2_biguasim_bluerovheavy.parm")"
SIM_VEHICLE="/home/teteu/ardupilot/Tools/autotest/sim_vehicle.py"

if [ -f "$EEPROM" ]; then
    echo "[t2_sitl_bluerovheavy] Deleting eeprom.bin to ensure clean params..."
    rm -f "$EEPROM"
fi

echo "[t2_sitl_bluerovheavy] Starting ArduSub SITL..."
cd /home/teteu/ardupilot/Tools/autotest

# Same -A "--rate 120" as t2_sitl_run_bluerov2.sh — ArduSub shares Copter's
# SCHEDULER_DEFAULT_LOOP_RATE=400 default (AP_Scheduler.cpp:44-47), unlike
# Rover's 50, so this is needed regardless of which ArduSub-based vehicle.
python3 "$SIM_VEHICLE" \
    -v ArduSub \
    --model JSON \
    --add-param-file="$PARAMS" \
    -A "--rate 120" \
    --no-mavproxy \
    -I0
