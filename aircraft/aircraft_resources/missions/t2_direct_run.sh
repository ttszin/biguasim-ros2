#!/bin/bash
# T2 Direct Runner — sem ArduPilot, sem MAVROS.
# Uso: bash t2_direct_run.sh [args extras para t2_direct_runner.py]
# Exemplo calibração: bash t2_direct_run.sh --viewport --calibrate
# Exemplo missão:     bash t2_direct_run.sh --viewport --water-x 8 --water-y 15 --water-z 13.0 --descent-z 11.0

source /opt/ros/jazzy/setup.bash

SCRIPT_DIR="$(dirname "$0")"

python3 "$SCRIPT_DIR/t2_direct_runner.py" "$@"
