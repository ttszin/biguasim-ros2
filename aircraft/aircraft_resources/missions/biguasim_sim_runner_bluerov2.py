"""Standalone BiguaSim runner for the BlueROV2 (ArduSub), bridged to real
ArduPilot Sub SITL — analogous to biguasim_sim_runner_blueboat.py's approach
for the BlueBoat/Rover, and to how biguasim_sim_runner.py does it for the
DjiMatrice.

No subclassing needed: ArduBiguaSimRunner's base run() loop (PWM in, motor
commands out, JSON state back to SITL) is already fully vehicle-agnostic —
see its own usage example in biguasim/src/biguasim/ardubridge/runner.py's
module docstring, which this script follows directly.

Usage:
    python3 biguasim_sim_runner_bluerov2.py --viewport
"""

from __future__ import annotations

import argparse

from biguasim.ardubridge import ArduBiguaSimRunner
from biguasim.ardubridge.vehicle import VEHICLE_REGISTRY

# Same x=25 water-crossing point already validated by t2_land_test.yaml/
# t2_hover_test.yaml/biguasim_sim_runner_blueboat.py. z=-1.0 (already
# submerged 1m below the surface at spawn) rather than the BlueBoat's z=0.2 —
# the BlueROV2 has no hull/buoyancy to float on the surface with, and this
# test's whole point is validating position hold while submerged. Starting
# point only, adjustable if live testing shows a spawn-splash/settle issue
# like the BlueBoat's did.
DEFAULT_LOCATION = [25.0, 0.0, -1.0]


def main() -> None:
    parser = argparse.ArgumentParser(description="BiguaSim BlueROV2 (ArduSub) SITL Runner")
    parser.add_argument("--viewport", action="store_true", help="Show Unreal Engine viewport")
    parser.add_argument("--ticks", type=int, default=200, help="Simulation ticks per second")
    parser.add_argument(
        "--location", nargs=3, type=float, default=DEFAULT_LOCATION,
        metavar=("X", "Y", "Z"),
        help=f"Agent start location in BiguaSim NWU metres (default: {DEFAULT_LOCATION})",
    )
    args = parser.parse_args()

    profile = VEHICLE_REGISTRY["BlueROV2"]
    scenario = ArduBiguaSimRunner.build_scenario(
        profile,
        package_name="SkyDive",
        world="Bridge",
        agent_name="bluerov0",
        location=args.location,
        rotation=[0.0, 0.0, 0.0],
        ticks_per_sec=args.ticks,
    )

    with ArduBiguaSimRunner(profile, scenario, show_viewport=args.viewport, verbose=True) as runner:
        runner.run()


if __name__ == "__main__":
    main()
