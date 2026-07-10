"""
BiguaSim simulation runner for T2 hybrid transition validation.

Replaces: Gazebo simulation + hydro_sensor_bridge.py

Runs the SkyDive/Bridge world (real water physics), reads the DepthSensor,
and streams pressure/ROV telemetry over UDP to the biguasim_bridge ROS2
node (Humble), which republishes /fcu/external_pressure, /nav_mode, and
/bluerov0/local_position, and mirrors ROV waypoint commands back here.

This script has NO ROS2 dependency by design: BiguaSim requires Python
>= 3.11, while ROS2 Humble's rclpy is built against Python 3.10 (Ubuntu
22.04) and cannot be imported in the same process. See aircraft_ws/src/
biguasim_bridge for the ROS2 side of this bridge.

Start order:
  1. bash t2_sitl_run.sh                                 (ArduCopter SITL)
  2. python3 biguasim_sim_runner.py [--viewport]          (this script)
  3. docker run --network host \
       -v $(pwd)/aircraft/t2_aircraft.yml.erb:/aas/t2_aircraft.yml.erb \
       --entrypoint bash aircraft-image \
       -c "tmuxinator start -p /aas/t2_aircraft.yml.erb"  (MAVROS +
     autopilot_interface + mission + biguasim_bridge, all ROS2 Humble —
     see aircraft/t2_aircraft.yml.erb)

ArduPilot receives pressure through the JSON SITL state (handled by
ArduBiguaSimRunner). The biguasim_bridge ROS2 node receives the same
pressure over UDP on --telemetry-port.
"""

from __future__ import annotations

import argparse
import json
import socket
import threading
import time

import numpy as np

from dataclasses import replace

from biguasim.ardubridge import ArduBiguaSimRunner, VehicleProfile
from biguasim.ardubridge.frame import depth_to_pressure
from biguasim.ardubridge.vehicle import VEHICLE_REGISTRY

# Perfil padrão do DjiMatrice (motor_mapping correto via sitl-test) com DepthSensor habilitado.
HYDRONE_HYBRID = replace(VEHICLE_REGISTRY["DjiMatrice"], include_depth_sensor=True)

# Cap telemetry UDP sends to ~50 Hz regardless of the (much faster) physics tick rate.
TELEMETRY_HZ = 50.0


class BiguaSimT2Runner(ArduBiguaSimRunner):
    """ArduBiguaSimRunner + UDP telemetry for the biguasim_bridge ROS2 node.

    The parent class already converts DepthSensor → pressure and sends it to
    ArduPilot via JSON SITL. This subclass taps into the same agent_state to
    forward pressure/ROV-position telemetry over UDP to a separate ROS2
    (Humble) process, and receives ROV waypoint commands back the same way.
    """

    def __init__(self, profile: VehicleProfile, scenario: dict,
                 spawn_location: list | None = None,
                 rov_agent: str = "bluerov0",
                 rov_hold: list | None = None,
                 bridge_host: str = "127.0.0.1",
                 telemetry_port: int = 9100,
                 rov_cmd_port: int = 9101,
                 **kwargs) -> None:
        super().__init__(profile, scenario, **kwargs)
        self._spawn_location = spawn_location
        self._rov_agent_name = rov_agent
        self._rov_hold = (rov_hold or [25.0, 0.0, -0.5]) + [0.0]
        self._rov_cmd_ros: list | None = None

        self._telemetry_addr = (bridge_host, telemetry_port)
        self._telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._rov_cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rov_cmd_sock.bind(("0.0.0.0", rov_cmd_port))
        self._rov_cmd_thread = threading.Thread(target=self._rov_cmd_loop, daemon=True)
        self._rov_cmd_thread.start()

        self._last_telemetry_send = 0.0
        print(
            f"BiguaSim T2 bridge ready -> UDP telemetry to {self._telemetry_addr}, "
            f"ROV cmd listening on 0.0.0.0:{rov_cmd_port}"
        )

    def _rov_cmd_loop(self) -> None:
        while True:
            try:
                data, _ = self._rov_cmd_sock.recvfrom(1024)
            except OSError:
                return
            try:
                payload = json.loads(data.decode("utf-8"))
                self._rov_cmd_ros = [payload["x"], payload["y"], payload["z"], 0.0]
            except (ValueError, KeyError):
                continue

    def _send_telemetry(self, pressure: float | None, rov_pos: list | None, sim_time: float) -> None:
        now = time.monotonic()
        if now - self._last_telemetry_send < (1.0 / TELEMETRY_HZ):
            return
        self._last_telemetry_send = now

        payload: dict = {"stamp": sim_time}
        if pressure is not None:
            payload["pressure_pa"] = pressure
        if rov_pos is not None:
            payload["rov_position"] = rov_pos

        self._telemetry_sock.sendto(json.dumps(payload).encode("utf-8"), self._telemetry_addr)

    def run(self) -> None:
        bridge = self._bridge
        env = self._env
        agent = self._agent_name
        dt = self._dt

        rov = self._rov_agent_name

        bridge.bind()
        motor_cmds = [0.0] * self._profile.num_motors
        rov_cmd = self._rov_hold
        env.step({agent: motor_cmds, rov: rov_cmd})

        if self._spawn_location is not None:
            env._agent.teleport(location=np.array(self._spawn_location, dtype=np.float32))
            env.step({agent: motor_cmds, rov: rov_cmd})

        raw = env.step({agent: motor_cmds, rov: rov_cmd})
        agent_state = raw[agent][0]
        sim_time = 0.0

        print(f"Running BiguaSim T2 bridge on '{agent}' + ROV '{rov}' (Ctrl-C to stop)...")
        try:
            while True:
                frame, pwm = bridge.receive_pwm()
                if frame is not None:
                    motor_cmds = bridge.pwm_to_motor_cmds(pwm, frame)

                rov_cmd = self._rov_cmd_ros if self._rov_cmd_ros is not None else self._rov_hold
                raw = env.step({agent: motor_cmds, rov: rov_cmd})
                agent_state = raw[agent][0]
                rov_state = raw[rov][0]
                sim_time += dt

                json_state = bridge.build_json_state(agent_state, sim_time)
                bridge.send_state(json_state)

                if json_state is not None and frame is not None and frame % 400 == 0:
                    pos = json_state["position"]
                    # pos: NED (north, east, down) relative to GPS origin
                    # BiguaSim x ≈ spawn_x + pos[0] (NED north)
                    # BiguaSim z ≈ spawn_z - pos[2] (NED down → up)
                    spawn_x = (self._spawn_location or [8.0])[0]
                    spawn_z = (self._spawn_location or [0.0, 0.0, 13.4])[2]
                    bsim_x = spawn_x + pos[0]
                    bsim_z = spawn_z - pos[2]
                    print(
                        f"  t={sim_time:.1f}s  NED=({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f})"
                        f"  bsim_x={bsim_x:.1f}  bsim_z={bsim_z:.1f}"
                        f"  motors={[f'{m:.0f}' for m in motor_cmds]}"
                    )

                pressure = None
                if "DepthSensor" in agent_state:
                    depth_val = agent_state["DepthSensor"]
                    z_up = float(depth_val[0]) if hasattr(depth_val, "__len__") else float(depth_val)
                    pressure = depth_to_pressure(z_up)

                rov_pos = None
                rov_loc = rov_state.get("LocationSensor")
                if rov_loc is not None:
                    rov_pos = [float(rov_loc[0]), float(rov_loc[1]), float(rov_loc[2])]

                self._send_telemetry(pressure, rov_pos, sim_time)

        except KeyboardInterrupt:
            print("Bridge stopped.")
        finally:
            bridge.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="BiguaSim T2 Simulation Runner")
    parser.add_argument("--viewport", action="store_true", help="Show Unreal Engine viewport")
    parser.add_argument("--port", type=int, default=9002, help="ArduPilot SITL UDP port")
    parser.add_argument("--ticks", type=int, default=200, help="Simulation ticks per second")
    parser.add_argument(
        "--location", nargs=3, type=float, default=[8.0, 0.0, 13.4],
        metavar=("X", "Y", "Z"),
        help="Agent start location in Biguasim NWU metres (default: 8 0 13.4)",
    )
    parser.add_argument(
        "--bridge-host", default="127.0.0.1",
        help="Host running the biguasim_bridge ROS2 node",
    )
    parser.add_argument(
        "--telemetry-port", type=int, default=9100,
        help="UDP port to send pressure/ROV telemetry to",
    )
    parser.add_argument(
        "--rov-cmd-port", type=int, default=9101,
        help="UDP port to listen for ROV waypoint commands on",
    )
    args = parser.parse_args()

    from biguasim.dynamics.agents import BlueROV2
    BlueROV2._params["kp_pos"] = 0.25

    rov_location = [25.0, 0.0, -0.5]

    scenario = ArduBiguaSimRunner.build_scenario(
        HYDRONE_HYBRID,
        package_name="SkyDive",
        world="Bridge",
        agent_name="hydrone0",
        location=args.location,
        rotation=[0.0, 0.0, 0.0],
        ticks_per_sec=args.ticks,
    )

    scenario["agents"].append({
        "agent_name": "bluerov0",
        "agent_type": "BlueROV2",
        "control_abstraction": "cmd_pos_yaw",
        "location": rov_location,
        "rotation": [0.0, 0.0, 0.0],
        "dynamics": {"batch_size": 1},
        "sensors": [
            {"sensor_type": "DynamicsSensor", "socket": "COM",
             "configuration": {"UseCOM": True, "UseRPY": False}},
            {"sensor_type": "LocationSensor", "socket": "COM", "configuration": {"Sigma": 0}},
        ],
    })

    with BiguaSimT2Runner(
        HYDRONE_HYBRID,
        scenario,
        spawn_location=args.location,
        rov_agent="bluerov0",
        rov_hold=rov_location,
        bridge_host=args.bridge_host,
        telemetry_port=args.telemetry_port,
        rov_cmd_port=args.rov_cmd_port,
        port=args.port,
        show_viewport=args.viewport,
    ) as runner:
        runner.run()


if __name__ == "__main__":
    main()
