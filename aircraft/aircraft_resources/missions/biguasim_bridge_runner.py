"""
BiguaSim ArduPilot bridge runner for T2 hybrid transition validation.

Replaces: Gazebo simulation + hydro_sensor_bridge.py

Runs the SkyDive/Bridge world (real water physics), reads the DepthSensor,
and publishes /fcu/external_pressure on ROS2 so validador_t2.py can detect
AERIAL_NAV ↔ AQUATIC_NAV transitions exactly as before.

Start order:
  1. python3 biguasim_bridge_runner.py [--viewport]   (this script)
  2. sim_vehicle.py -v ArduCopter -L RATBeach --console --map \
       -f quadx --model JSON:127.0.0.1 --no-mavproxy
  3. ros2 launch mavros apm.launch fcu_url:=udp://:14550@
  4. python3 validador_t2.py
  5. ros2 run mission mission_node --conops t2_biguasim_mission.yaml

ArduPilot receives pressure through the JSON SITL state (handled by ArduBiguaSimRunner).
ROS2 receives the same pressure on /fcu/external_pressure (published by this script).
"""

from __future__ import annotations

import argparse
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image

from dataclasses import replace

from biguasim.ardubridge import ArduBiguaSimRunner, VehicleProfile
from biguasim.ardubridge.frame import depth_to_pressure
from biguasim.ardubridge.vehicle import VEHICLE_REGISTRY

# Perfil padrão do DjiMatrice (motor_mapping correto via sitl-test) com DepthSensor habilitado.
HYDRONE_HYBRID = replace(VEHICLE_REGISTRY["DjiMatrice"], include_depth_sensor=True)


class BiguaSimT2Runner(ArduBiguaSimRunner):
    """ArduBiguaSimRunner + ROS2 publisher for /fcu/external_pressure.

    The parent class already converts DepthSensor → pressure and sends it to
    ArduPilot via JSON SITL. This subclass taps into the same agent_state to
    publish FluidPressure on ROS2, feeding validador_t2.py.
    """

    def __init__(self, profile: VehicleProfile, scenario: dict,
                 spawn_location: list | None = None,
                 rov_agent: str = "bluerov0",
                 rov_hold: list | None = None,
                 **kwargs) -> None:
        super().__init__(profile, scenario, **kwargs)
        self._spawn_location = spawn_location
        self._rov_agent_name = rov_agent
        self._rov_hold = (rov_hold or [25.0, 0.0, -0.5]) + [0.0]
        self._rov_cmd_ros: list | None = None

        if not rclpy.ok():
            rclpy.init()
        self._ros_node = Node("biguasim_pressure_bridge")

        self._press_pub = self._ros_node.create_publisher(
            FluidPressure, "/fcu/external_pressure", 10
        )
        self._nav_mode_pub = self._ros_node.create_publisher(
            String, "/nav_mode", 10
        )
        self._rov_pos_pub = self._ros_node.create_publisher(
            Point, "/bluerov0/local_position", 10
        )
        self._cam_pub = self._ros_node.create_publisher(
            Image, "/biguasim/camera/image", 10
        )
        self._ros_node.create_subscription(
            Point, "/bluerov0/cmd_pos_yaw", self._rov_cmd_cb, 10
        )

        # Inline nav-mode validator (mirrors validador_t2.py logic)
        self._nav_mode = "AERIAL_NAV"
        self._PRESSURE_ENTER = 102500.0   # ~12 cm depth
        self._PRESSURE_EXIT  = 101500.0   # ~1.7 cm depth
        self._DEBOUNCE_SECS  = 0.5        # reduced from 1.5 s: AQUATIC_NAV fires at ~-1 m
        self._candidate_mode: str | None = None
        self._candidate_since: float | None = None
        # publish initial mode
        self._nav_mode_pub.publish(String(data=self._nav_mode))

        self._ros_thread = threading.Thread(
            target=lambda: rclpy.spin(self._ros_node), daemon=True
        )
        self._ros_thread.start()
        self._ros_node.get_logger().info(
            "BiguaSim pressure bridge ready → /fcu/external_pressure  /bluerov0/local_position"
        )

    def _rov_cmd_cb(self, msg: Point) -> None:
        self._rov_cmd_ros = [msg.x, msg.y, msg.z, 0.0]

    def _update_nav_mode(self, pressure: float) -> None:
        """Hysteresis + debounce pressure→nav_mode, publishes /nav_mode on transitions."""
        import time
        now = time.monotonic()

        if self._nav_mode == "AERIAL_NAV" and pressure > self._PRESSURE_ENTER:
            target = "AQUATIC_NAV"
        elif self._nav_mode == "AQUATIC_NAV" and pressure < self._PRESSURE_EXIT:
            target = "AERIAL_NAV"
        else:
            self._candidate_mode = None
            self._candidate_since = None
            return

        if self._candidate_mode != target:
            self._candidate_mode = target
            self._candidate_since = now
            return

        if (now - self._candidate_since) >= self._DEBOUNCE_SECS:
            self._nav_mode = target
            self._candidate_mode = None
            self._candidate_since = None
            self._nav_mode_pub.publish(String(data=self._nav_mode))
            self._ros_node.get_logger().info(
                f"[nav_mode] transition → {self._nav_mode}  (pressure={pressure:.0f} Pa)"
            )

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
                rov_state  = raw[rov][0]
                sim_time += dt

                json_state = bridge.build_json_state(agent_state, sim_time)
                bridge.send_state(json_state)

                if json_state is not None and frame is not None and frame % 400 == 0:
                    q = json_state["quaternion"]
                    pos = json_state["position"]
                    # pos: NED (north, east, down) relative to GPS origin
                    # BiguaSim x ≈ spawn_x + pos[0] (NED north)
                    # BiguaSim z ≈ spawn_z - pos[2] (NED down → up)
                    spawn_x = (self._spawn_location or [8.0])[0]
                    spawn_z = (self._spawn_location or [0.0, 0.0, 13.4])[2]
                    bsim_x = spawn_x + pos[0]
                    bsim_z = spawn_z - pos[2]
                    nav = self._nav_mode
                    print(
                        f"  t={sim_time:.1f}s  NED=({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f})"
                        f"  bsim_x={bsim_x:.1f}  bsim_z={bsim_z:.1f}"
                        f"  nav={nav}  motors={[f'{m:.0f}' for m in motor_cmds]}"
                    )

                if "DepthSensor" in agent_state:
                    depth_val = agent_state["DepthSensor"]
                    z_up = float(depth_val[0]) if hasattr(depth_val, "__len__") else float(depth_val)
                    pressure = depth_to_pressure(z_up)
                    msg = FluidPressure()
                    msg.header.stamp = self._ros_node.get_clock().now().to_msg()
                    msg.fluid_pressure = pressure
                    self._press_pub.publish(msg)
                    self._update_nav_mode(pressure)

                rov_loc = rov_state.get("LocationSensor")
                if rov_loc is not None:
                    pt = Point()
                    pt.x, pt.y, pt.z = float(rov_loc[0]), float(rov_loc[1]), float(rov_loc[2])
                    self._rov_pos_pub.publish(pt)

                cam_data = agent_state.get("Camera")
                if cam_data is not None:
                    rgb = cam_data[:, :, :3]  # drop alpha channel
                    msg = Image()
                    msg.header.stamp = self._ros_node.get_clock().now().to_msg()
                    msg.header.frame_id = "CameraSocket"
                    msg.height = rgb.shape[0]
                    msg.width = rgb.shape[1]
                    msg.encoding = "rgb8"
                    msg.step = msg.width * 3
                    msg.data = rgb.tobytes()
                    self._cam_pub.publish(msg)

        except KeyboardInterrupt:
            print("Bridge stopped.")
        finally:
            bridge.close()
            self._ros_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description="BiguaSim T2 Bridge Runner")
    parser.add_argument("--viewport", action="store_true", help="Show Unreal Engine viewport")
    parser.add_argument("--port", type=int, default=9002, help="ArduPilot SITL UDP port")
    parser.add_argument("--ticks", type=int, default=200, help="Simulation ticks per second")
    parser.add_argument(
        "--location", nargs=3, type=float, default=[8.0, 0.0, 13.4],
        metavar=("X", "Y", "Z"),
        help="Agent start location in Biguasim NWU metres (default: 8 0 13.4)",
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
    scenario["agents"][0]["sensors"].append({
        "sensor_type": "RGBCamera",
        "sensor_name": "Camera",
        "socket": "CameraSocket",
        "rotation": [0.0, 90.0, 0.0],  # pitch +90° → nadir (looking straight down)
        "configuration": {"CaptureWidth": 256, "CaptureHeight": 256},
    })

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
        port=args.port,
        show_viewport=args.viewport,
    ) as runner:
        runner.run()


if __name__ == "__main__":
    main()
