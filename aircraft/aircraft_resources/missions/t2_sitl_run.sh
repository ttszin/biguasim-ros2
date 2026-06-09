#!/bin/bash
# Starts ArduCopter SITL for Biguasim (JSON interface on port 9002, MAVLink on TCP 5760).
# Deletes eeprom.bin first so --defaults params are always respected.
# Usage: bash t2_sitl_run.sh

EEPROM="/home/teteu/ardupilot/Tools/autotest/eeprom.bin"
PARAMS="/home/teteu/ardupilot/Tools/autotest/t2_params.parm"
SIM_VEHICLE="/home/teteu/ardupilot/Tools/autotest/sim_vehicle.py"

if [ -f "$EEPROM" ]; then
    echo "[t2_sitl] Deleting eeprom.bin to ensure clean params..."
    rm -f "$EEPROM"
fi

echo "[t2_sitl] Starting ArduCopter SITL..."
cd /home/teteu/ardupilot/Tools/autotest

python3 "$SIM_VEHICLE" \
    -v ArduCopter \
    --model JSON \
    --add-param-file="$PARAMS" \
    --no-mavproxy \
    -I0
