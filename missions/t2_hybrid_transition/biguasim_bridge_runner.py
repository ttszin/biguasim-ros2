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
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure

from biguasim.ardubridge import ArduBiguaSimRunner, VehicleProfile
from biguasim.ardubridge.frame import depth_to_pressure
from biguasim.ardubridge.vehicle import _unipolar_pwm

# DjiMatrice profile extended with DepthSensor for hybrid water-entry detection.
# Identical to the stock DjiMatrice except include_depth_sensor=True.
HYDRONE_HYBRID = VehicleProfile(
    name="DjiMatrice",
    num_motors=4,
    motor_mapping=[2, 0, 3, 1],
    motor_signs=[1, 1, 1, 1],
    pwm_converters=[_unipolar_pwm(592.4)] * 4,
    control_abstraction="cmd_motor_speeds",
    ardupilot_vehicle="ArduCopter",
    sitl_args=(
        "sim_vehicle.py -v ArduCopter -L RATBeach --console --map "
        "-f quadx --model JSON:127.0.0.1 --no-mavproxy"
    ),
    include_depth_sensor=True,
    warmup_frames=500,  # 500 frames @ 200 Hz = 2.5s para EKF convergir antes de aplicar motores
)


class BiguaSimT2Runner(ArduBiguaSimRunner):
    """ArduBiguaSimRunner + ROS2 publisher for /fcu/external_pressure.

    The parent class already converts DepthSensor → pressure and sends it to
    ArduPilot via JSON SITL. This subclass taps into the same agent_state to
    publish FluidPressure on ROS2, feeding validador_t2.py.
    """

    def __init__(self, profile: VehicleProfile, scenario: dict,
                 spawn_location: list | None = None, **kwargs) -> None:
        super().__init__(profile, scenario, **kwargs)
        self._spawn_location = spawn_location
        if not rclpy.ok():
            rclpy.init()
        self._ros_node = Node("biguasim_pressure_bridge")
        self._press_pub = self._ros_node.create_publisher(
            FluidPressure, "/fcu/external_pressure", 10
        )
        self._ros_thread = threading.Thread(
            target=lambda: rclpy.spin(self._ros_node), daemon=True
        )
        self._ros_thread.start()
        self._ros_node.get_logger().info(
            "BiguaSim pressure bridge ready → /fcu/external_pressure"
        )

    def run(self) -> None:
        bridge = self._bridge
        env = self._env
        agent = self._agent_name
        dt = self._dt

        bridge.bind()
        motor_cmds = [0.0] * self._profile.num_motors
        env.step(motor_cmds)

        # Teleport força o spawn no local correto independente do que o mundo faz
        # durante os pre_start_steps (o Bridge world tem posição fixa no UE5).
        # env.agents usa chave "hydrone0-id0" (sufixo interno de batch), então
        # usamos env._agent que referencia diretamente o agente principal.
        if self._spawn_location is not None:
            env._agent.teleport(location=np.array(self._spawn_location, dtype=np.float32))
            env.step(motor_cmds)

        raw = env.step(motor_cmds)
        agent_state = raw[agent][0]
        sim_time = 0.0

        print(f"Running BiguaSim T2 bridge on '{agent}' (Ctrl-C to stop)...")
        try:
            while True:
                frame, pwm = bridge.receive_pwm()
                if frame is not None:
                    motor_cmds = bridge.pwm_to_motor_cmds(pwm, frame)

                raw = env.step(motor_cmds)
                agent_state = raw[agent][0]
                sim_time += dt

                json_state = bridge.build_json_state(agent_state, sim_time)
                bridge.send_state(json_state)

                if "DepthSensor" in agent_state:
                    depth_val = agent_state["DepthSensor"]
                    z_up = float(depth_val[0]) if hasattr(depth_val, "__len__") else float(depth_val)
                    msg = FluidPressure()
                    msg.header.stamp = self._ros_node.get_clock().now().to_msg()
                    msg.fluid_pressure = depth_to_pressure(z_up)
                    self._press_pub.publish(msg)

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

    scenario = ArduBiguaSimRunner.build_scenario(
        HYDRONE_HYBRID,
        package_name="SkyDive",
        world="Bridge",
        agent_name="hydrone0",
        location=args.location,
        rotation=[0.0, 0.0, 0.0],
        ticks_per_sec=args.ticks,
    )

    with BiguaSimT2Runner(
        HYDRONE_HYBRID,
        scenario,
        spawn_location=args.location,
        port=args.port,
        show_viewport=args.viewport,
    ) as runner:
        runner.run()


if __name__ == "__main__":
    main()
