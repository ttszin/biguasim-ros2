"""Workaround runner for BlueROVHeavy: bridges ArduPilot to BiguaSim via
cmd_vel_yaw instead of cmd_motor_speeds.

Why: BiguaSim's SpawnAgentCommand.cpp segfaults (SpawnedAgent assertion,
SIGSEGV) whenever an agent is spawned with control_abstraction=
"cmd_motor_speeds" — confirmed for both BlueROV2 and BlueROVHeavy, and
confirmed NOT to be about sensors, cache, viewport, depth, or memory
pressure (see biguasim_bridge/README.md, "Position-hold for BlueROV2
(ArduSub)"). "cmd_vel_yaw" spawns and runs fine (same blueprint, same
sensors) — this script routes ArduPilot's real PWM output through that
abstraction instead.

This is explicitly an APPROXIMATION, not equivalent to the real
cmd_motor_speeds path: BiguaSim's own cmd_vel_yaw handler (uuv.py) runs a
real P-controller (velocity error -> desired force -> thruster forces via
TM_to_f -> motor speeds), so real thrust/mass dynamics are still exercised
-- but there are now two stacked control loops (ArduSub's own EKF/position
controller, then BiguaSim's velocity controller) instead of one, and the
PWM->velocity translation below is a rough reconstruction, not an exact
motor-mixing inverse (BiguaSim's real per-motor thruster geometry for the
Heavy frame isn't available from this repo). Concretely:
  - heave (vz): mean of the 4 vertical motor commands (MOT1-4, profile
    indices 0-3) -- high confidence, these are unambiguously heave/roll/
    pitch thrusters.
  - surge (vx): mean of the 4 horizontal motor commands (MOT5-8, indices
    4-7) -- reasonable when all four are pushing roughly together, but
    doesn't separate the sway or yaw components those same 4 vectored
    thrusters also produce.
  - sway (vy): not reconstructed (0.0) -- not separable from surge/yaw
    without the real thruster angle geometry.
  - yaw: not driven from PWM at all -- yaw_delta is always 0.0, so
    BiguaSim's own cmd_vel_yaw yaw-hold PID keeps whatever heading the
    vehicle already has.
Good enough to see whether GUIDED-mode corrections produce a stabilizing
response at all; not a substitute for the real per-motor bridge once
BiguaSim's SpawnAgentCommand bug is fixed upstream.

Usage:
    python3 t2_bluerovheavy_velbridge_runner.py --viewport
"""

from __future__ import annotations

import argparse
import dataclasses

from biguasim.ardubridge import ArduBiguaSimRunner
from biguasim.ardubridge.vehicle import VEHICLE_REGISTRY

DEFAULT_LOCATION = [25.0, 0.0, -1.0]

# Vertical (heave/roll/pitch) vs horizontal (surge/sway/yaw) motor index
# split, per vehicle.py's own "4 vertical (MOT1-4) + 4 horizontal (MOT5-8)"
# comment for BlueROVHeavy.
_VERTICAL_MOTOR_IDXS = (0, 1, 2, 3)
_HORIZONTAL_MOTOR_IDXS = (4, 5, 6, 7)

# Rough scale from normalized per-motor command (~[-1, 1]) to a commanded
# velocity in m/s. Not calibrated against BiguaSim's real thrust curve --
# picked to be a plausible slow ROV speed at full command, tune by
# observation if the response looks too sluggish/aggressive.
_HEAVE_GAIN = 0.5
_SURGE_GAIN = 0.5


def pwm_to_vel_yaw_cmd(motor_cmds: list) -> list:
    """Approximate translation from per-motor commands to [vx, vy, vz, yaw_delta_deg]."""
    vertical = [motor_cmds[i] for i in _VERTICAL_MOTOR_IDXS]
    horizontal = [motor_cmds[i] for i in _HORIZONTAL_MOTOR_IDXS]
    vz = -(sum(vertical) / len(vertical)) * _HEAVE_GAIN  # NWU: positive command -> descend
    vx = (sum(horizontal) / len(horizontal)) * _SURGE_GAIN
    vy = 0.0
    yaw_delta_deg = 0.0
    return [vx, vy, vz, yaw_delta_deg]


def main() -> None:
    parser = argparse.ArgumentParser(description="BiguaSim BlueROVHeavy cmd_vel_yaw workaround runner")
    parser.add_argument("--viewport", action="store_true", help="Show Unreal Engine viewport")
    parser.add_argument("--ticks", type=int, default=200, help="Simulation ticks per second")
    parser.add_argument(
        "--location", nargs=3, type=float, default=DEFAULT_LOCATION,
        metavar=("X", "Y", "Z"),
        help=f"Agent start location in BiguaSim NWU metres (default: {DEFAULT_LOCATION})",
    )
    args = parser.parse_args()

    # The one load-bearing change: cmd_vel_yaw instead of cmd_motor_speeds.
    # dataclasses.replace() returns a copy -- doesn't touch VEHICLE_REGISTRY.
    profile = dataclasses.replace(VEHICLE_REGISTRY["BlueROVHeavy"], control_abstraction="cmd_vel_yaw")

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
        # Reimplements ArduBiguaSimRunner.run()'s loop (can't reuse it as-is:
        # it always calls bridge.pwm_to_motor_cmds(), which returns a
        # num_motors-length vector -- wrong shape for cmd_vel_yaw's 4-value
        # goal). Reaches into runner's own bridge/env instead of duplicating
        # their construction.
        bridge = runner._bridge
        env = runner._env
        agent = runner._agent_name
        dt = runner._dt

        bridge.bind()
        cmd = [0.0, 0.0, 0.0, 0.0]
        raw = env.step(cmd)
        sim_time = 0.0

        print(f"Running BlueROVHeavy SITL bridge via cmd_vel_yaw workaround (Ctrl-C to stop)...")
        try:
            while True:
                frame, pwm = bridge.receive_pwm()
                if frame is None:
                    continue
                motor_cmds = bridge.pwm_to_motor_cmds(pwm, frame)
                cmd = pwm_to_vel_yaw_cmd(motor_cmds)

                raw = env.step(cmd)
                agent_state = raw[agent][0]
                sim_time += dt

                json_state = bridge.build_json_state(agent_state, sim_time)
                bridge.send_state(json_state)

                if frame is not None and frame % 200 == 0:
                    print(f"  t={sim_time:.2f}s frame={frame} cmd_vel_yaw={[f'{v:.3f}' for v in cmd]}")
        except KeyboardInterrupt:
            print("Bridge stopped.")
        finally:
            bridge.close()


if __name__ == "__main__":
    main()
