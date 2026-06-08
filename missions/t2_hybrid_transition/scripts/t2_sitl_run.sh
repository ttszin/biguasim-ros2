#!/bin/bash
# Starts ArduCopter SITL for BiguaSim (JSON interface on port 9002, MAVLink on TCP 5760).
# Deletes eeprom.bin first so --defaults params are always respected.
#
# Usage: bash t2_sitl_run.sh [path/to/t2_params.parm]
#
# Requires ArduPilot installed. Set ARDUPILOT_PATH env var if not at ~/ardupilot.

ARDUPILOT_PATH="${ARDUPILOT_PATH:-$HOME/ardupilot}"
AUTOTEST="$ARDUPILOT_PATH/Tools/autotest"
EEPROM="$AUTOTEST/eeprom.bin"
PARAMS="${1:-$AUTOTEST/t2_params.parm}"
SIM_VEHICLE="$AUTOTEST/sim_vehicle.py"

if [ ! -f "$SIM_VEHICLE" ]; then
    echo "[t2_sitl] ERROR: sim_vehicle.py not found at $SIM_VEHICLE"
    echo "         Set ARDUPILOT_PATH to your ArduPilot install directory."
    exit 1
fi

if [ -f "$EEPROM" ]; then
    echo "[t2_sitl] Deleting eeprom.bin to ensure clean params..."
    rm -f "$EEPROM"
fi

echo "[t2_sitl] Starting ArduCopter SITL (JSON @ port 9002, MAVLink TCP @ 5760)..."
cd "$AUTOTEST"

python3 "$SIM_VEHICLE" \
    -v ArduCopter \
    --model JSON \
    --add-param-file="$PARAMS" \
    --no-mavproxy \
    -I0
