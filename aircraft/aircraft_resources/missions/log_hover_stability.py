#!/usr/bin/env python3
"""Standalone hover-stability logger — no changes to mission_node.py or
ardupilot_interface.cpp needed, since /mavros/local_position/odom and
/mavros/imu/data are already published by default (not in this stack's
apm_pluginlists.yaml denylist). Run alongside t2_hover_test.yaml (or any
other mission) in its own terminal to capture and summarize how well the
vehicle holds position/attitude.

Also subscribes to /mavros/global_position/local (same nav_msgs/Odometry
shape) as a fallback position source: confirmed live that ArduSub's MAVROS
instance never publishes /mavros/local_position/odom at all (no error, just
silence — IMU and every GPS-derived topic work fine), while
/mavros/global_position/local (also EKF/GPS-derived, just republished by
MAVROS's global_position plugin under a different topic) does. Copter/Rover
are unaffected — they already publish local_position/odom, which is
preferred whenever both are available.

Usage:
    python3 log_hover_stability.py --duration 180 --settle 20 --out hover_baseline.csv
"""

from __future__ import annotations

import argparse
import csv
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def _quat_to_euler_deg(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """Standard aerospace-sequence (roll, pitch, yaw) from a quaternion, in
    degrees — implemented directly (no tf_transformations dependency, not
    otherwise used anywhere in this repo's Python side).
    """
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class HoverStabilityLogger(Node):
    def __init__(self, out_path: str, duration: float, settle: float):
        super().__init__('hover_stability_logger')
        self._duration = duration
        self._settle = settle
        self._rows = []

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # Same topics/QoS ardupilot_interface.cpp already subscribes to (see
        # its local_position_odom_callback) — reused here read-only, no
        # interference with the existing node. global_position/local is a
        # fallback for vehicles (confirmed: ArduSub) that never publish
        # local_position/odom — see module docstring.
        self._odom_sub = self.create_subscription(Odometry, '/mavros/local_position/odom', self._on_odom, qos)
        self._global_local_sub = self.create_subscription(Odometry, '/mavros/global_position/local', self._on_global_local, qos)
        self._imu_sub = self.create_subscription(Imu, '/mavros/imu/data', self._on_imu, qos)

        self._last_odom = None
        self._last_global_local = None
        self._last_imu = None

        self._out_path = out_path
        self._csv_file = open(out_path, 'w', newline='')
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow(['t', 'x', 'y', 'z', 'vx', 'vy', 'vz',
                                'roll_deg', 'pitch_deg', 'yaw_deg',
                                'roll_rate', 'pitch_rate', 'yaw_rate'])

        # Wall-clock start, independent of when/whether odom+imu data ever
        # arrives — the duration/timeout check below must not be gated behind
        # receiving data, or a topic-name/QoS/DDS-discovery problem (confirmed
        # live: cross-distro Jazzy-host/Humble-container discovery reported
        # "Topic type hash: INVALID" and never delivered messages to a
        # host-side subscriber) hangs this script forever with no error.
        self._t0 = self.get_clock().now().nanoseconds / 1e9
        self._no_data_warned = False

        self._timer = self.create_timer(0.1, self._tick)  # 10 Hz sample rate
        self.get_logger().info(f"Logging to '{out_path}' for {duration}s (first {settle}s excluded from summary stats).")

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom = msg

    def _on_global_local(self, msg: Odometry) -> None:
        self._last_global_local = msg

    def _on_imu(self, msg: Imu) -> None:
        self._last_imu = msg

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        t = now - self._t0

        odom = self._last_odom or self._last_global_local
        if odom is None or self._last_imu is None:
            if t >= self._duration:
                self.get_logger().error(
                    f"No data received on /mavros/local_position/odom, "
                    f"/mavros/global_position/local, or /mavros/imu/data after "
                    f"{self._duration}s — check topic names/QoS or run this from inside the same "
                    f"ROS2 distro as mavros (cross-distro host<->container DDS discovery can "
                    f"report topics as visible without ever delivering messages)."
                )
                self._csv_file.close()
                rclpy.try_shutdown()
            return

        p = odom.pose.pose.position
        v = odom.twist.twist.linear
        q = odom.pose.pose.orientation
        roll, pitch, yaw = _quat_to_euler_deg(q.x, q.y, q.z, q.w)
        av = self._last_imu.angular_velocity

        row = (t, p.x, p.y, p.z, v.x, v.y, v.z, roll, pitch, yaw, av.x, av.y, av.z)
        self._rows.append(row)
        self._writer.writerow([f'{val:.4f}' for val in row])
        self._csv_file.flush()  # regular-file stdout is block-buffered — without this,
        # nothing is visible on disk (even the header) until close(), making it impossible
        # to check progress on a still-running process (confirmed live: `wc -l` showed 0
        # lines despite the process actively consuming CPU and genuinely receiving data).

        if t >= self._duration:
            self._finish()

    def _finish(self) -> None:
        self._timer.cancel()
        self._csv_file.close()
        settled = [r for r in self._rows if r[0] >= self._settle]
        if len(settled) < 2:
            self.get_logger().warn("Not enough samples after the settle window to summarize.")
        else:
            summary = _summarize(settled)
            self.get_logger().info(
                "Hover stability summary (post-settle, n=%d samples):\n"
                "  horizontal pos stddev: %.3f m\n"
                "  vertical   pos stddev: %.3f m\n"
                "  peak |roll|:  %.2f deg\n"
                "  peak |pitch|: %.2f deg"
                % (len(settled), summary['horiz_std'], summary['vert_std'],
                   summary['peak_roll'], summary['peak_pitch'])
            )
        self.get_logger().info(f"Raw samples saved to '{self._out_path}'.")
        rclpy.try_shutdown()


def _summarize(rows: list[tuple]) -> dict:
    n = len(rows)
    xs = [r[1] for r in rows]
    ys = [r[2] for r in rows]
    zs = [r[3] for r in rows]
    rolls = [r[7] for r in rows]
    pitches = [r[8] for r in rows]

    def stddev(vals):
        mean = sum(vals) / n
        return math.sqrt(sum((v - mean) ** 2 for v in vals) / n)

    horiz_std = math.sqrt(stddev(xs) ** 2 + stddev(ys) ** 2)
    return {
        'horiz_std': horiz_std,
        'vert_std': stddev(zs),
        'peak_roll': max(abs(r) for r in rolls),
        'peak_pitch': max(abs(p) for p in pitches),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Log MAVROS odom/IMU during a hover to measure position/attitude stability")
    parser.add_argument("--duration", type=float, default=180.0, help="Total seconds to log (match the mission's hover wait).")
    parser.add_argument("--settle", type=float, default=20.0, help="Seconds of post-takeoff transient excluded from the summary.")
    parser.add_argument("--out", default="hover_stability.csv", help="CSV output path.")
    args = parser.parse_args()

    rclpy.init()
    node = HoverStabilityLogger(args.out, args.duration, args.settle)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
