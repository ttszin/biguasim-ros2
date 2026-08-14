"""Standalone BiguaSim runner for the BlueROVHeavy (ArduSub) — same
ArduBiguaSimRunner-direct pattern as biguasim_sim_runner_bluerov2.py.

Purely diagnostic for now: used to test whether BiguaSim/Unreal's
deterministic hang on BlueROV2 spawn (see biguasim_bridge/README.md,
"Position-hold for BlueROV2 (ArduSub)") is specific to that vehicle's own
asset, or a broader ArduSub/BiguaSim issue. BlueROVHeavy shares ArduSub and
include_depth_sensor=True with BlueROV2, so:
  - if this does NOT hang, that points squarely at BlueROV2BP's own asset
    (material/mesh), not anything Sub-related or common BiguaSim/ArduSub
    infrastructure.
  - if this DOES hang identically, that broadens the suspect to something
    shared by both ROV blueprints (still not DepthSensor specifically —
    already ruled out for BlueROV2 by removing it and reproducing the same
    hang — but possibly a shared base class/material both ROV blueprints
    inherit from).

Usage:
    python3 biguasim_sim_runner_bluerovheavy.py --viewport
"""

from __future__ import annotations

import argparse

from biguasim.ardubridge import ArduBiguaSimRunner
from biguasim.ardubridge.vehicle import VEHICLE_REGISTRY

# Same water-crossing spawn point already used for BlueROV2/BlueBoat.
DEFAULT_LOCATION = [25.0, 0.0, -1.0]


def main() -> None:
    parser = argparse.ArgumentParser(description="BiguaSim BlueROVHeavy (ArduSub) SITL Runner")
    parser.add_argument("--viewport", action="store_true", help="Show Unreal Engine viewport")
    parser.add_argument("--ticks", type=int, default=200, help="Simulation ticks per second")
    parser.add_argument(
        "--location", nargs=3, type=float, default=DEFAULT_LOCATION,
        metavar=("X", "Y", "Z"),
        help=f"Agent start location in BiguaSim NWU metres (default: {DEFAULT_LOCATION})",
    )
    args = parser.parse_args()

    profile = VEHICLE_REGISTRY["BlueROVHeavy"]
    scenario = ArduBiguaSimRunner.build_scenario(
        profile,
        package_name="SkyDive",
        world="Bridge",
        agent_name="bluerovheavy0",
        location=args.location,
        rotation=[0.0, 0.0, 0.0],
        ticks_per_sec=args.ticks,
    )

    with ArduBiguaSimRunner(profile, scenario, show_viewport=args.viewport, verbose=True) as runner:
        runner.run()


if __name__ == "__main__":
    main()
