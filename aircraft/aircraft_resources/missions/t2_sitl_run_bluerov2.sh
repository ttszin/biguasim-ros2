#!/bin/bash
# Starts ArduSub SITL for the BlueROV2, standalone (JSON interface on port 9002,
# MAVLink on TCP 5760 — same ports as t2_sitl_run.sh's ArduCopter instance and
# t2_sitl_run_blueboat.sh's Rover instance, since these are meant to run one at
# a time, never together).
# Deletes eeprom.bin first so --defaults params are always respected.
# Usage: bash t2_sitl_run_bluerov2.sh

EEPROM="/home/teteu/ardupilot/Tools/autotest/eeprom.bin"
PARAMS="$(realpath "$(dirname "$0")/t2_biguasim_bluerov2.parm")"
SIM_VEHICLE="/home/teteu/ardupilot/Tools/autotest/sim_vehicle.py"

if [ -f "$EEPROM" ]; then
    echo "[t2_sitl_bluerov2] Deleting eeprom.bin to ensure clean params..."
    rm -f "$EEPROM"
fi

echo "[t2_sitl_bluerov2] Starting ArduSub SITL..."
cd /home/teteu/ardupilot/Tools/autotest

# -A "--rate 120", same as t2_sitl_run.sh's ArduCopter instance — unlike Rover,
# which didn't need this. Confirmed in
# ardupilot/libraries/AP_Scheduler/AP_Scheduler.cpp:44-47:
# SCHEDULER_DEFAULT_LOOP_RATE is 400 for Copter/Heli/ArduSub (Rover is the odd
# one out at 50), so ArduSub has the exact same loop-rate expectation Copter
# had — the same 120Hz fix (margin under BiguaSim's observed ~133Hz real
# delivery rate, see t2_biguasim.parm's SCHED_LOOP_RATE comment) is applied
# here from the start rather than rediscovering it live.
python3 "$SIM_VEHICLE" \
    -v ArduSub \
    --model JSON \
    --add-param-file="$PARAMS" \
    -A "--rate 120" \
    --no-mavproxy \
    -I0
