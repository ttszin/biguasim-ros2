#!/bin/bash
# Starts ArduCopter SITL for Biguasim (JSON interface on port 9002, MAVLink on TCP 5760).
# Deletes eeprom.bin first so --defaults params are always respected.
# Usage: bash t2_sitl_run.sh

EEPROM="/home/teteu/ardupilot/Tools/autotest/eeprom.bin"
PARAMS="$(realpath "$(dirname "$0")/t2_biguasim.parm")"
SIM_VEHICLE="/home/teteu/ardupilot/Tools/autotest/sim_vehicle.py"

if [ -f "$EEPROM" ]; then
    echo "[t2_sitl] Deleting eeprom.bin to ensure clean params..."
    rm -f "$EEPROM"
fi

echo "[t2_sitl] Starting ArduCopter SITL..."
cd /home/teteu/ardupilot/Tools/autotest

# -A "--rate 120": sets SIM_RATE_HZ=120 on the arducopter binary (confirmed in
# ardupilot/libraries/AP_HAL_SITL/SITL_cmdline.cpp's -r/--rate handling) — the
# rate ArduCopter's own scheduler EXPECTS physics/FDM updates to arrive at.
# Default is 400Hz (Copter's compiled-in assumption). Must be kept equal to
# t2_biguasim.parm's SCHED_LOOP_RATE — see that param's comment for why both
# (two independent rates, confirmed live) need to match, and why 120: the
# real BiguaSim delivery rate measured live is ~133Hz, identically with and
# without --viewport (not a rendering/GPU-load bottleneck) and stable across
# very different GPU temperatures (not thermal throttling either) — 120
# leaves margin under that observed ceiling, not BiguaSim's requested/default
# 200 (biguasim_sim_runner.py's --ticks).
python3 "$SIM_VEHICLE" \
    -v ArduCopter \
    --model JSON \
    --add-param-file="$PARAMS" \
    -A "--rate 120" \
    --no-mavproxy \
    -I0
