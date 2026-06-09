import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import String


class ValidadorT2(Node):
    def __init__(self):
        super().__init__('validador_t2')

        # Hysteresis thresholds (Pa)
        # Entry: ~12 cm depth  →  P_ATM + 1000*9.81*0.12 ≈ 102502 Pa
        # Exit:  ~1.7 cm depth →  P_ATM + 1000*9.81*0.017 ≈ 101492 Pa
        # The wide gap (~700 Pa / ~7 cm) prevents mode toggling near the surface.
        self.PRESSURE_ENTER_WATER = 102500.0
        self.PRESSURE_EXIT_WATER = 101500.0

        # Require this many consecutive seconds of consistent readings before switching
        self.DEBOUNCE_SECS = 1.5

        self.nav_mode = 'AERIAL_NAV'
        self._candidate_mode = None
        self._candidate_since = None

        self.pressure_sub = self.create_subscription(
            FluidPressure,
            '/fcu/external_pressure',
            self.pressure_callback,
            10)

        self.nav_mode_pub = self.create_publisher(String, '/nav_mode', 10)

        self._publish_mode()
        self.get_logger().info(f'Validator started. Initial mode: {self.nav_mode}')

    def pressure_callback(self, msg):
        pressure = msg.fluid_pressure
        now = self.get_clock().now()

        # Determine which mode the current reading suggests
        if self.nav_mode == 'AERIAL_NAV' and pressure > self.PRESSURE_ENTER_WATER:
            target = 'AQUATIC_NAV'
        elif self.nav_mode == 'AQUATIC_NAV' and pressure < self.PRESSURE_EXIT_WATER:
            target = 'AERIAL_NAV'
        else:
            # Reading is within the hysteresis band — reset debounce and do nothing
            self._candidate_mode = None
            self._candidate_since = None
            return

        # Start debounce timer when a new candidate mode appears
        if self._candidate_mode != target:
            self._candidate_mode = target
            self._candidate_since = now
            return

        # Confirm transition only after DEBOUNCE_SECS of consistent readings
        elapsed = (now - self._candidate_since).nanoseconds / 1e9
        if elapsed >= self.DEBOUNCE_SECS:
            self.nav_mode = target
            self._candidate_mode = None
            self._candidate_since = None
            self._publish_mode()
            self.get_logger().info(f'>>> [VALIDATOR] Mode transition → {self.nav_mode}')

    def _publish_mode(self):
        self.nav_mode_pub.publish(String(data=self.nav_mode))


def main(args=None):
    rclpy.init(args=args)
    node = ValidadorT2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
