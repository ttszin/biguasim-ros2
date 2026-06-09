import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from px4_msgs.msg import VehicleLocalPosition
from sensor_msgs.msg import FluidPressure

class HydroSensorBridge(Node):
    def __init__(self):
        super().__init__('hydro_sensor_bridge')

        # Tank 5x5 at world ENU East=10, North=0 (impalpable_greyness.sdf)
        # PX4 VehicleLocalPosition uses NED: x=North, y=East, z=Down
        self.tank_center_east  = 10.0
        self.tank_center_north = 0.0
        self.tank_size = 5.0
        self.min_east  = self.tank_center_east  - self.tank_size / 2.0  # 7.5
        self.max_east  = self.tank_center_east  + self.tank_size / 2.0  # 12.5
        self.min_north = self.tank_center_north - self.tank_size / 2.0  # -2.5
        self.max_north = self.tank_center_north + self.tank_size / 2.0  # 2.5

        # Water surface raised to 2.3m: SITL min altitude ~1.9m, so drone IS submerged at ~1.9m
        self.z_water_surface = 2.3

        self.P_ATM = 101325.0
        self.RHO   = 1000.0
        self.G     = 9.81

        drone_id = os.environ.get('DRONE_ID', '1')
        topic = f'/Drone{drone_id}/fmu/out/vehicle_local_position'

        # PX4 DDS publishes with BEST_EFFORT
        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.pose_sub = self.create_subscription(
            VehicleLocalPosition, topic, self.pose_callback, px4_qos)

        self.press_pub = self.create_publisher(FluidPressure, '/fcu/external_pressure', 10)

        self.get_logger().info(f'Hydrostatic pressure sensor STARTED. Listening on {topic}')

    def pose_callback(self, msg):
        north    = msg.x        # NED x = North
        east     = msg.y        # NED y = East
        altitude = -msg.z       # NED z = Down → altitude = -z

        is_inside = (
            self.min_east  <= east  <= self.max_east and
            self.min_north <= north <= self.max_north
        )

        press_msg = FluidPressure()
        press_msg.header.stamp = self.get_clock().now().to_msg()

        if is_inside and altitude < self.z_water_surface:
            depth = self.z_water_surface - altitude
            press_msg.fluid_pressure = self.P_ATM + self.RHO * self.G * depth
        else:
            press_msg.fluid_pressure = self.P_ATM

        self.press_pub.publish(press_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HydroSensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
