#!/bin/bash
# ArduSub SITL for the Stage D (underwater planner) tests: identical to missions/t2_sitl_run_bluerov2.sh except for the
# SITL home altitude. sim_vehicle.py defaults to CMAC (584 m AMSL); ArduSub's SITL barometer feeds that altitude to a
# water model (AP_Baro_SITL: SimpleUnderWaterAtmosphere(-alt)), which yields a NEGATIVE absolute pressure
# (SCALED_PRESSURE ~ -57640 hPa), the EKF then ignores the barometer and the depth estimate freezes while the ROV
# dives (found 2026-09-24, see BIGUASIM_NOTES.md). Home at altitude 0 = the water surface makes the pressure physical.
# Usage: bash rov_sitl.sh
EEPROM="/home/teteu/ardupilot/Tools/autotest/eeprom.bin"
PARAMS="$(realpath "$(dirname "$0")/../../missions/t2_biguasim_bluerov2.parm")"
SIM_VEHICLE="/home/teteu/ardupilot/Tools/autotest/sim_vehicle.py"
[ -f "$EEPROM" ] && rm -f "$EEPROM"
cd /home/teteu/ardupilot/Tools/autotest
python3 "$SIM_VEHICLE" -v ArduSub --model JSON --add-param-file="$PARAMS" -A "--rate 120" --no-mavproxy -I0 \
    -l -35.363261,149.165230,0,353
