"""Standalone BiguaSim runner for the BlueBoat (Rover), bridged to real
ArduPilot Rover SITL — unlike biguasim_sim_runner.py's own `blueboat0`
agent (a decorative, non-SITL `cmd_pos_yaw`-controlled prop used as a
landing target), this runs the BlueBoat as the actual ArduPilot-controlled
vehicle, analogous to how biguasim_sim_runner.py does it for the DjiMatrice.

No subclassing needed: ArduBiguaSimRunner's base run() loop (PWM in, motor
commands out, JSON state back to SITL) is already fully vehicle-agnostic —
see its own usage example in biguasim/src/biguasim/ardubridge/runner.py's
module docstring, which this script follows directly.

Usage:
    python3 biguasim_sim_runner_blueboat.py --viewport
"""

from __future__ import annotations

import argparse

from biguasim.ardubridge import ArduBiguaSimRunner
from biguasim.ardubridge.vehicle import VEHICLE_REGISTRY

# x=25 (bsim_x = spawn_x(8) + north 17) is the same water-crossing point
# already validated by t2_land_test.yaml/t2_hover_test.yaml — reused here so
# the BlueBoat spawns directly in open water, not on the dry bridge deck
# (x=8). z=0.2 matches biguasim_sim_runner.py's own tuned --boat-z default
# for the decorative BlueBoat prop (first guess at minimizing spawn-splash
# oscillation — see that script's --boat-z help text).
DEFAULT_LOCATION = [25.0, 0.0, 0.2]


def main() -> None:
    parser = argparse.ArgumentParser(description="BiguaSim BlueBoat (Rover) SITL Runner")
    parser.add_argument("--viewport", action="store_true", help="Show Unreal Engine viewport")
    parser.add_argument("--ticks", type=int, default=200, help="Simulation ticks per second")
    parser.add_argument(
        "--location", nargs=3, type=float, default=DEFAULT_LOCATION,
        metavar=("X", "Y", "Z"),
        help=f"Agent start location in BiguaSim NWU metres (default: {DEFAULT_LOCATION})",
    )
    args = parser.parse_args()

    profile = VEHICLE_REGISTRY["BlueBoat"]
    scenario = ArduBiguaSimRunner.build_scenario(
        profile,
        package_name="SkyDive",
        world="Bridge",
        agent_name="blueboat0",
        location=args.location,
        rotation=[0.0, 0.0, 0.0],
        ticks_per_sec=args.ticks,
    )

    with ArduBiguaSimRunner(profile, scenario, show_viewport=args.viewport, verbose=True) as runner:
        runner.run()


if __name__ == "__main__":
    main()
