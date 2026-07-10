import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

import os
import argparse
import threading
import random
import time
import yaml

from action_msgs.msg import GoalStatus
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from mavros_msgs.msg import VfrHud, ExtendedState
from geometry_msgs.msg import Point
from vision_msgs.msg import Detection2DArray
try:
    from px4_msgs.msg import VehicleGlobalPosition, AirspeedValidated
    _PX4_AVAILABLE = True
except ImportError:
    _PX4_AVAILABLE = False

try:
    from ground_system_msgs.msg import SwarmObs
except ImportError:
    pass
from state_sharing.msg import SharedState
from autopilot_interface_msgs.action import Land, Offboard, Takeoff, Orbit
from autopilot_interface_msgs.srv import SetSpeed, SetReposition

class MissionNode(Node):
    def __init__(self, mission_file):
        super().__init__('mission_node')

        self.mission_plan = []
        self.get_logger().info(f"Loading conops from: {mission_file}")

        if not os.path.isabs(mission_file):
            mission_file = os.path.join('/aas/aircraft_resources/missions/', mission_file)
        try:
            with open(mission_file, 'r') as f:
                data = yaml.safe_load(f)
                self.mission_plan = data.get('steps', [])
                self.get_logger().info(f"Loaded {len(self.mission_plan)} steps.")
        except Exception as e:
            self.get_logger().error(f"Failed to load mission file: {e}")
            self.mission_plan = []

        self.mission_step = 0 # Track the advancement of the mission
        self.last_executed_step = -1 # To avoid re-executing the same step
        self.active_mission_goal_handle = None # Hold the goal handle of the active action
        self.wait_start_time = None # For "wait" mission steps

        self.own_drone_id = None
        drone_id_str = os.environ.get('DRONE_ID') # Get id from ENV VAR
        if drone_id_str is None:
            self.get_logger().info("DRONE_ID environment variable not set.")
        else:
            try:
                self.own_drone_id = int(drone_id_str)
            except ValueError:
                self.get_logger().info(f"Could not parse DRONE_ID='{drone_id_str}' as an integer.")

        self.data_lock = threading.Lock()
        # MAVROS data
        self.lat = None
        self.lon = None
        self.alt_msl = None
        self.heading = None
        self.airspeed = None
        # Perception data
        self.yolo_detections = None
        # self.ground_tracks = None
        # State sharing
        self.active_state_sharing_subs = {}
        self.drone_states = {}
        self.STALE_DRONE_TIMEOUT_SEC = 5.0 # Time after which we prune a drone from drone_states
        # Navigation mode (published by validador_t2)
        self.current_nav_mode = 'AERIAL_NAV'
        self.nav_mode_wait_target = None
        self.nav_mode_wait_start = None
        self.nav_mode_wait_timeout = 30.0
        # Bool topic wait (e.g. /bluerov0/tour_done)
        self.bool_topic_wait_topic = None
        self.bool_topic_wait_start = None
        self.bool_topic_wait_timeout = 300.0
        self.bool_topic_wait_flag = False
        # Land-complete detection
        self.landed_state = 0  # ExtendedState: 0=UNDEFINED, 1=ON_GROUND, 2=IN_AIR, 3=TAKEOFF, 4=LANDING
        self.land_complete_waiting = False
        self.land_complete_start = None
        self.land_complete_timeout = 60.0
        self.landing_surface = None  # 'platform' or 'water', set when land_complete fires
        # Altitude-stability fallback: fires when alt_msl is stable for 3s (ArduPilot land detector
        # may not fire in BiguaSim when landing on a rigid platform in direct LAND mode).
        self.land_alt_prev = None
        self.land_alt_stable_start = None
        # descend_to_water: vel.z=-2 m/s (GPS-free) until AQUATIC_NAV, then auto-float
        self.descend_to_water_active = False
        self.descend_to_water_start = None
        self.descend_to_water_timeout = 180.0
        self.descend_to_water_last_send = None
        # ascend_from_water: vel.z=+2 m/s (GPS-free) until AERIAL_NAV, refreshed every 10s
        self.ascend_from_water_active = False
        self.ascend_from_water_start = None
        self.ascend_from_water_timeout = 120.0
        self.ascend_from_water_last_send = None
        # ascend_to_altitude: vel.z=+2 m/s until alt_msl >= target, refreshed every 10s
        self.ascend_to_alt_active = False
        self.ascend_to_alt_target_msl = 0.0
        self.ascend_to_alt_start = None
        self.ascend_to_alt_timeout = 120.0
        self.ascend_to_alt_last_send = None
        # vision_land: closed-loop centering + descent over a detected marker/target,
        # driven by /detections (self.yolo_detections). Reuses SetReposition's
        # GPS-free velocity mode (altitude>10) with east/north as centering velocity
        # and vertical_velocity as a variable descent rate.
        self.vision_land_active = False
        self.vision_land_target_class_id = "shape_target"
        self.vision_land_start = None
        self.vision_land_timeout = 120.0
        self.vision_land_last_send = None
        self.vision_land_centered_since = None
        self.vision_land_centered_threshold_deg = 5.0
        self.vision_land_min_confirm_secs = 2.0
        self.vision_land_land_alt_msl = 571.5
        self.vision_land_kp_horizontal = 0.05  # m/s per degree of azimuth/elevation error
        self.vision_land_max_horizontal_vel = 2.0
        self.vision_land_descend_vel = 1.0  # m/s, applied (as a descent) only once centered
        # wait_to_reach_position: advance when EKF lat/lon is within threshold of target north/east
        self.reach_position_active = False
        self.reach_position_target_north = 0.0
        self.reach_position_threshold = 3.0
        self.reach_position_start = None
        self.reach_position_timeout = 60.0
        # home position — captured on first GPS fix, used for north/east distance calculations
        self.home_lat = None
        self.home_lon = None
        # ROV waypoint tracking
        self.rov_position = None
        self.rov_waypoint_waiting = False
        self.rov_waypoint_target = None
        self.rov_waypoint_threshold = 1.5
        self.rov_waypoint_timeout = 120.0
        self.rov_waypoint_start = None

        # Create a reentrant callback groups to allow callbacks to run in parallel
        self.subscriber_callback_group = ReentrantCallbackGroup()
        self.timer_callback_group = ReentrantCallbackGroup()
        self.action_callback_group = ReentrantCallbackGroup()
        self.service_callback_group = ReentrantCallbackGroup()

        # Create a QoS profile for the subscribers
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        # PX4 subscribers (only when px4_msgs is available)
        if _PX4_AVAILABLE:
            self.create_subscription( # 100Hz
                VehicleGlobalPosition, 'fmu/out/vehicle_global_position', self.px4_global_position_callback,
                self.qos_profile, callback_group=self.subscriber_callback_group)
            self.create_subscription( # 10Hz
                AirspeedValidated, '/fmu/out/airspeed_validated', self.airspeed_validated_callback,
                self.qos_profile, callback_group=self.subscriber_callback_group)
        # MAVROS subscribers
        self.create_subscription( # 4Hz
            NavSatFix, '/mavros/global_position/global', self.mavros_global_position_callback,
            self.qos_profile, callback_group=self.subscriber_callback_group)
        self.create_subscription( # 4Hz
            VfrHud, '/mavros/vfr_hud', self.vfr_hud_callback,
            self.qos_profile, callback_group=self.subscriber_callback_group)
        # Perception subscribers
        self.create_subscription( # 15Hz
            Detection2DArray, '/detections', self.yolo_detections_callback,
            self.qos_profile, callback_group=self.subscriber_callback_group)
        # Navigation mode (from validador_t2 via hydro_sensor_bridge)
        self.create_subscription(
            String, '/nav_mode', self.nav_mode_callback,
            10, callback_group=self.subscriber_callback_group)
        # ArduPilot land_complete signal
        self.create_subscription(
            ExtendedState, '/mavros/extended_state', self.extended_state_callback,
            10, callback_group=self.subscriber_callback_group)
        # ROV tour completion
        from std_msgs.msg import Bool as BoolMsg
        self.create_subscription(
            BoolMsg, '/bluerov0/tour_done', self._tour_done_cb,
            10, callback_group=self.subscriber_callback_group)
        # ROV position feedback and command publisher
        self.create_subscription(
            Point, '/bluerov0/local_position', self._rov_pos_cb,
            10, callback_group=self.subscriber_callback_group)
        self._rov_cmd_pub = self.create_publisher(Point, '/bluerov0/cmd_pos_yaw', 10)
        # self.create_subscription( # 1Hz
        #     SwarmObs, '/tracks', self.ground_tracks_callback,
        #     self.qos_profile, callback_group=self.subscriber_callback_group)

        # Timed callbacks
        self.discover_drones_timer = self.create_timer(
            5.0, # 0.2Hz
            self.discover_drones_callback,
            callback_group=self.timer_callback_group
        )
        self.stale_check_timer = self.create_timer(
            2.0, # 0.5Hz
            self.check_stale_drones_callback,
            callback_group=self.timer_callback_group
        )
        self.printout_timer = self.create_timer(
            3.0, # 0.33Hz
            self.printout_callback,
            callback_group=self.timer_callback_group
        )
        self.conops_timer = self.create_timer(
            1.0, # 1Hz
            self.conops_callback,
            callback_group=self.timer_callback_group
        )

        # Actions — use absolute paths when drone_id is known, consistent with services
        if self.own_drone_id is not None:
            ns = f'/Drone{self.own_drone_id}'
            self._takeoff_client = ActionClient(self, Takeoff, f'{ns}/takeoff_action', callback_group=self.action_callback_group)
            self._land_client = ActionClient(self, Land, f'{ns}/land_action', callback_group=self.action_callback_group)
            self._orbit_client = ActionClient(self, Orbit, f'{ns}/orbit_action', callback_group=self.action_callback_group)
            self._offboard_client = ActionClient(self, Offboard, f'{ns}/offboard_action', callback_group=self.action_callback_group)
        else:
            self._takeoff_client = ActionClient(self, Takeoff, 'takeoff_action', callback_group=self.action_callback_group)
            self._land_client = ActionClient(self, Land, 'land_action', callback_group=self.action_callback_group)
            self._orbit_client = ActionClient(self, Orbit, 'orbit_action', callback_group=self.action_callback_group)
            self._offboard_client = ActionClient(self, Offboard, 'offboard_action', callback_group=self.action_callback_group)

        # Services
        if self.own_drone_id is not None:
            self._speed_client = self.create_client(
                SetSpeed, f'/Drone{self.own_drone_id}/set_speed',
                callback_group=self.service_callback_group
            )
            self._reposition_client = self.create_client(
                SetReposition, f'/Drone{self.own_drone_id}/set_reposition',
                callback_group=self.service_callback_group
            )
        else:
            self._speed_client = None
            self._reposition_client = None
            self.get_logger().info("DRONE_ID not set, service clients not created.")

    def px4_global_position_callback(self, msg): # Mutally exclusive with mavros_global_position_callback
        with self.data_lock:
            self.lat = msg.lat
            self.lon = msg.lon
            self.alt_msl = msg.alt

    def airspeed_validated_callback(self, msg): # Mutally exclusive with vfr_hud_callback
        with self.data_lock:
            self.airspeed = msg.true_airspeed_m_s

    def mavros_global_position_callback(self, msg):  # Mutally exclusive with px4_global_position_callback
        with self.data_lock:
            self.lat = msg.latitude
            self.lon = msg.longitude
            # MAVROS can publish an all-zero placeholder before the FCU has a real GPS
            # fix; skip capturing home from that, otherwise wait_to_reach_position's
            # distance math (which relies on home_lat/home_lon) is permanently wrong.
            position_valid = not (abs(msg.latitude) < 1e-6 and abs(msg.longitude) < 1e-6)
            if self.home_lat is None and position_valid:
                self.home_lat = msg.latitude
                self.home_lon = msg.longitude

    def vfr_hud_callback(self, msg): # Mutally exclusive with airspeed_validated_callback
        with self.data_lock:
            self.alt_msl = msg.altitude
            self.heading = msg.heading
            self.airspeed = msg.airspeed

    def yolo_detections_callback(self, msg):
        with self.data_lock:
            self.yolo_detections = msg

    def nav_mode_callback(self, msg):
        with self.data_lock:
            self.current_nav_mode = msg.data
        # React in subscriber (not timer) so there is zero polling delay.
        if self.descend_to_water_active and msg.data == 'AQUATIC_NAV':
            req = SetReposition.Request()
            req.north = 0.0
            req.east = 0.0
            req.altitude = -0.5  # triggers vel.z=0 (float at surface)
            # Use no-advance variant: mission_step must not be incremented by service_response_callback
            self._call_service_no_advance(self._reposition_client, req)
            self.descend_to_water_active = False
            self.descend_to_water_start = None
            self.get_logger().info("descend_to_water: AQUATIC_NAV detected, switched to float.")
            self.mission_step += 1
        elif self.ascend_from_water_active and msg.data == 'AERIAL_NAV':
            self.ascend_from_water_active = False
            self.ascend_from_water_start = None
            self.get_logger().info("ascend_from_water: AERIAL_NAV detected, ascent complete.")
            self.mission_step += 1

    def extended_state_callback(self, msg):
        with self.data_lock:
            self.landed_state = msg.landed_state

    def _tour_done_cb(self, msg):
        if msg.data:
            with self.data_lock:
                self.bool_topic_wait_flag = True

    def _rov_pos_cb(self, msg):
        with self.data_lock:
            self.rov_position = (msg.x, msg.y, msg.z)

    def discover_drones_callback(self):
        topic_prefix = '/state_sharing_drone_'
        current_topics_and_types = self.get_topic_names_and_types() # This still re-discovers dead Zenoh topics but data won't be added to drone_states if they are not published
        for topic_name, msg_types in current_topics_and_types:
            if topic_name.startswith(topic_prefix) and topic_name not in self.active_state_sharing_subs:
                if 'state_sharing/msg/SharedState' in msg_types:
                    try:
                        topic_drone_id = int(topic_name.replace(topic_prefix, ''))
                        if topic_drone_id == self.own_drone_id:
                            continue # Ignore self
                    except ValueError:
                        continue # Skip if the topic name is malformed
                    self.get_logger().info(f"Discovered new drone: subscribing to {topic_name}")
                    sub = self.create_subscription( # 1Hz
                        SharedState,
                        topic_name,
                        self.state_sharing_callback,
                        self.qos_profile,
                        callback_group=self.subscriber_callback_group
                    )
                    self.active_state_sharing_subs[topic_name] = sub # Store the subscriber

    def check_stale_drones_callback(self):
        now = self.get_clock().now()
        stale_ids = []
        with self.data_lock:
            for drone_id, (last_msg, last_seen_time) in self.drone_states.items():
                duration = now - last_seen_time
                if duration.nanoseconds / 1e9 > self.STALE_DRONE_TIMEOUT_SEC:
                    stale_ids.append(drone_id)
            for drone_id in stale_ids:
                self.get_logger().info(f"Drone {drone_id} timed out. Removing.")
                # Remove the subscriber
                topic_name_to_remove = f"/state_sharing_drone_{drone_id}"
                if topic_name_to_remove in self.active_state_sharing_subs:
                    sub = self.active_state_sharing_subs.pop(topic_name_to_remove)
                    self.destroy_subscription(sub)
                # Remove the data
                self.drone_states.pop(drone_id, None)

    def state_sharing_callback(self, msg):
        # A single callback for all drone state topics
        with self.data_lock:
            now = self.get_clock().now()
            self.drone_states[msg.drone_id] = (msg, now)

    # def ground_tracks_callback(self, msg):
    #     with self.data_lock:
    #         self.ground_tracks = msg

    def printout_callback(self):
        with self.data_lock: # Copy with lock
            mission_step = self.mission_step
            lat = self.lat
            lon = self.lon
            alt_msl = self.alt_msl
            yolo_detections = self.yolo_detections
            states_copy = self.drone_states.copy()
            # ground_tracks = self.ground_tracks
        now_seconds = self.get_clock().now().nanoseconds / 1e9
        output = f"\nCurrent node time: {now_seconds:.2f} seconds\n"
        output += f"Mission step: {mission_step}\n"
        lat_str = f"{lat:.5f}" if lat is not None else "N/A"
        lon_str = f"{lon:.5f}" if lon is not None else "N/A"
        alt_str = f"{alt_msl:.2f}" if alt_msl is not None else "N/A"
        output += f"Global Position:\n  lat: {lat_str} lon: {lon_str} alt: {alt_str} (msl)\n"
        #
        if yolo_detections and yolo_detections.detections:
            output += "YOLO Detections:\n"
            for detection in yolo_detections.detections:
                for result in detection.results:
                    output += f"  Label: {result.hypothesis.class_id} - conf: {result.hypothesis.score:.2f}\n"
        else:
            output += "YOLO Detections: [No data]\n"
        #
        if not states_copy:
            output += "State Sharing: [No data]\n"
        else:
            now_seconds = self.get_clock().now().nanoseconds / 1e9
            output += "State Sharing:\n"
            for drone_id, (state_msg, last_seen_time) in sorted(states_copy.items()):
                seconds_ago = now_seconds - (last_seen_time.nanoseconds / 1e9)
                output += (f"  Id {drone_id}, lat: {state_msg.latitude_deg:.5f} lon: {state_msg.longitude_deg:.5f}, "
                        f"alt: {state_msg.altitude_m:.2f} (px4: msl, ap: ell.), hdg: {state_msg.heading_deg:.1f}deg, "
                        f"vel: [{state_msg.vx:.1f}, {state_msg.vy:.1f}, {state_msg.vz:.1f}]"
                        f"(seen {seconds_ago:.1f}s ago)\n")
        #
        # if ground_tracks and ground_tracks.tracks:
        #     output += "Ground Tracks:\n"
        #     for track in ground_tracks.tracks:
        #         output += f"  Id {track.id}, lat: {track.latitude_deg:.5f} lon: {track.longitude_deg:.5f} alt (msl): {track.altitude_m:.2f}\n"
        # else:
        #     output += "Ground Tracks: [No data]\n"
        #
        self.get_logger().info(output)

    def send_goal(self, client, goal_msg):
        if self.active_mission_goal_handle is not None:
            self.get_logger().info("An action is already in progress. Cannot send new goal.")
            return False
        if not client.server_is_ready():
            self.get_logger().warn('Action server not ready, retrying next cycle...')
            return False
        self.get_logger().info('Sending goal request...')
        send_goal_future = client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.mission_step = -1
            return
        self.active_mission_goal_handle = goal_handle
        self.get_logger().info('Goal accepted! Waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.active_mission_goal_handle = None # Clear the handle
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Action succeeded! Result: {result.success}")
            self.mission_step += 1 # Advance the mission step
        else:
            self.get_logger().info(f"Action failed with status: {status}")
            self.mission_step = -1

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Received action feedback: {feedback_msg.feedback.message}")

    def call_service(self, server, request):
        if server is None or not server.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service not available.')
            return
        future = server.call_async(request)
        future.add_done_callback(self.service_response_callback)

    def _call_service_no_advance(self, server, request):
        """Send a service request without auto-advancing mission_step."""
        if server is None or not server.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service not available.')
            return
        future = server.call_async(request)
        future.add_done_callback(self._service_no_advance_callback)

    def _service_no_advance_callback(self, future):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(f"Service call failed (no-advance): {response.message}")
        except Exception as e:
            self.get_logger().error(f'Service call failed (no-advance): {e}')

    def service_response_callback(self, future):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(f"Service call failed: {response.message}")
                self.mission_step = -1
                return
            self.get_logger().info(f'Service call successful: {response.success}')
            self.mission_step += 1 # Advance the mission step
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self.mission_step = -1

    def conops_callback(self):
        # If an ROS Action is running, do nothing
        if self.active_mission_goal_handle is not None:
            return

        # If waiting for a bool topic (e.g. tour_done)
        if self.bool_topic_wait_topic is not None:
            with self.data_lock:
                flag = self.bool_topic_wait_flag
            if flag:
                self.get_logger().info(f"Bool topic '{self.bool_topic_wait_topic}' received True.")
                self.bool_topic_wait_topic = None
                self.bool_topic_wait_flag = False
                self.mission_step += 1
            else:
                elapsed = (self.get_clock().now() - self.bool_topic_wait_start).nanoseconds / 1e9
                if elapsed > self.bool_topic_wait_timeout:
                    self.get_logger().error(f"Timeout waiting for '{self.bool_topic_wait_topic}'.")
                    self.mission_step = -1
                    self.bool_topic_wait_topic = None
            return

        # If waiting for the ROV to reach a waypoint
        if self.rov_waypoint_waiting:
            with self.data_lock:
                pos = self.rov_position
                target = self.rov_waypoint_target
                threshold = self.rov_waypoint_threshold
            if pos is not None and target is not None:
                dist = ((pos[0]-target[0])**2 + (pos[1]-target[1])**2 + (pos[2]-target[2])**2)**0.5
                if dist < threshold:
                    self.get_logger().info(f"ROV reached waypoint (dist={dist:.2f} m)")
                    self.rov_waypoint_waiting = False
                    self.rov_waypoint_target = None
                    self.mission_step += 1
                    return
            elapsed = (self.get_clock().now() - self.rov_waypoint_start).nanoseconds / 1e9
            if elapsed > self.rov_waypoint_timeout:
                self.get_logger().error("Timeout waiting for ROV waypoint.")
                self.mission_step = -1
                self.rov_waypoint_waiting = False
            return

        # If waiting for a nav mode transition, poll current mode
        if self.nav_mode_wait_target is not None:
            with self.data_lock:
                current_mode = self.current_nav_mode
            if current_mode == self.nav_mode_wait_target:
                self.get_logger().info(f"Nav mode '{self.nav_mode_wait_target}' confirmed.")
                self.nav_mode_wait_target = None
                self.nav_mode_wait_start = None
                self.mission_step += 1
            else:
                elapsed = (self.get_clock().now() - self.nav_mode_wait_start).nanoseconds / 1e9
                if elapsed > self.nav_mode_wait_timeout:
                    self.get_logger().error(
                        f"Timeout waiting for nav mode '{self.nav_mode_wait_target}'.")
                    self.mission_step = -1
                    self.nav_mode_wait_target = None
            return

        # If waiting for land_complete (ExtendedState.landed_state == ON_GROUND)
        if self.land_complete_waiting:
            with self.data_lock:
                ls = self.landed_state
                nav_mode = self.current_nav_mode
                current_alt = self.alt_msl
            if ls == 1:  # LANDED_STATE_ON_GROUND (physical touchdown, platform or rigid surface)
                surface = 'water' if nav_mode == 'AQUATIC_NAV' else 'platform'
                self.landing_surface = surface
                self.land_complete_waiting = False
                self.land_alt_prev = None
                self.land_alt_stable_start = None
                self.get_logger().info(f"Land complete (ON_GROUND). Surface: {surface}.")
                self.mission_step += 1
            elif nav_mode == 'AQUATIC_NAV':  # floating on water surface (buoyancy prevents ON_GROUND)
                self.landing_surface = 'water'
                self.land_complete_waiting = False
                self.land_alt_prev = None
                self.land_alt_stable_start = None
                self.get_logger().info("Land complete (AQUATIC_NAV surface). Surface: water.")
                self.mission_step += 1
            else:
                # Fallback: detect landing via altitude stability.
                # ArduPilot land detector may not fire in BiguaSim on rigid platforms.
                if current_alt is not None:
                    if self.land_alt_stable_start is None:
                        self.land_alt_stable_start = self.get_clock().now()
                        self.land_alt_prev = current_alt
                    elif abs(current_alt - self.land_alt_prev) >= 0.1:
                        # Cumulative drift from window-start reference exceeds threshold — reset.
                        # Comparing against the window-start ref (not adjacent samples) catches
                        # slow descents where each 1-second step is individually < 0.1 m.
                        self.land_alt_stable_start = self.get_clock().now()
                        self.land_alt_prev = current_alt
                    stable_elapsed = (self.get_clock().now() - self.land_alt_stable_start).nanoseconds / 1e9
                    if stable_elapsed >= 3.0:
                        surface = 'platform'
                        self.landing_surface = surface
                        self.land_complete_waiting = False
                        self.land_alt_prev = None
                        self.land_alt_stable_start = None
                        self.get_logger().info(
                            f"Land complete (alt stable at {current_alt:.1f} MSL). Surface: {surface}.")
                        self.mission_step += 1
                        return
                elapsed = (self.get_clock().now() - self.land_complete_start).nanoseconds / 1e9
                if elapsed > self.land_complete_timeout:
                    self.get_logger().error("Timeout waiting for land_complete.")
                    self.mission_step = -1
                    self.land_complete_waiting = False
            return

        # descend_to_water: re-send vel.z=-2 every 10s to keep GUID_TIMEOUT alive.
        # mission_step advance happens in nav_mode_callback when AQUATIC_NAV fires.
        if self.descend_to_water_active:
            elapsed = (self.get_clock().now() - self.descend_to_water_start).nanoseconds / 1e9
            if elapsed > self.descend_to_water_timeout:
                self.get_logger().error("Timeout waiting for AQUATIC_NAV in descend_to_water.")
                self.descend_to_water_active = False
                self.descend_to_water_start = None
                self.mission_step = -1
                return
            last_send_elapsed = (self.get_clock().now() - self.descend_to_water_last_send).nanoseconds / 1e9
            if last_send_elapsed >= 10.0:
                req = SetReposition.Request()
                req.north = 0.0
                req.east = 0.0
                req.altitude = -10.0  # < -1 → vel.z = -2 m/s (GPS-free descent)
                self._call_service_no_advance(self._reposition_client, req)
                self.descend_to_water_last_send = self.get_clock().now()
                self.get_logger().info("descend_to_water: re-sending descent command.")
            return

        # ascend_from_water: re-send vel.z=+2 every 10s to keep GUID_TIMEOUT alive.
        # mission_step advance happens in nav_mode_callback when AERIAL_NAV fires.
        if self.ascend_from_water_active:
            elapsed = (self.get_clock().now() - self.ascend_from_water_start).nanoseconds / 1e9
            if elapsed > self.ascend_from_water_timeout:
                self.get_logger().error("Timeout waiting for AERIAL_NAV in ascend_from_water.")
                self.ascend_from_water_active = False
                self.ascend_from_water_start = None
                self.mission_step = -1
                return
            last_send_elapsed = (self.get_clock().now() - self.ascend_from_water_last_send).nanoseconds / 1e9
            if last_send_elapsed >= 10.0:
                req = SetReposition.Request()
                req.north = 0.0
                req.east = 0.0
                req.altitude = 15.0  # > 10 → vel.z = +2 m/s (GPS-free ascent)
                req.vertical_velocity = 2.0
                self._call_service_no_advance(self._reposition_client, req)
                self.ascend_from_water_last_send = self.get_clock().now()
                self.get_logger().info("ascend_from_water: re-sending ascent command.")
            return

        if self.ascend_to_alt_active:
            with self.data_lock:
                current_alt = self.alt_msl
            if current_alt is not None and current_alt >= self.ascend_to_alt_target_msl:
                self.ascend_to_alt_active = False
                self.get_logger().info(
                    f"ascend_to_altitude: reached {current_alt:.1f} MSL (target {self.ascend_to_alt_target_msl:.1f}).")
                self.mission_step += 1
                return
            if self.ascend_to_alt_timeout > 0:
                elapsed = (self.get_clock().now() - self.ascend_to_alt_start).nanoseconds / 1e9
                if elapsed > self.ascend_to_alt_timeout:
                    self.get_logger().error("Timeout in ascend_to_altitude.")
                    self.ascend_to_alt_active = False
                    self.mission_step = -1
                    return
            last_send_elapsed = (self.get_clock().now() - self.ascend_to_alt_last_send).nanoseconds / 1e9
            if last_send_elapsed >= 10.0:
                req = SetReposition.Request()
                req.north = 0.0
                req.east = 0.0
                req.altitude = 15.0
                req.vertical_velocity = 2.0
                self._call_service_no_advance(self._reposition_client, req)
                self.ascend_to_alt_last_send = self.get_clock().now()
                alt_str = f"{current_alt:.1f}" if current_alt is not None else "N/A"
                self.get_logger().info(
                    f"ascend_to_altitude: re-sending vel.z+2 (alt={alt_str} MSL, target={self.ascend_to_alt_target_msl:.1f}).")
            return

        if self.vision_land_active:
            elapsed = (self.get_clock().now() - self.vision_land_start).nanoseconds / 1e9
            if elapsed > self.vision_land_timeout:
                self.get_logger().error(
                    f"Timeout waiting to center on '{self.vision_land_target_class_id}' in vision_land.")
                self.vision_land_active = False
                self.mission_step = -1
                return

            with self.data_lock:
                detections_msg = self.yolo_detections
                current_alt = self.alt_msl

            azimuth_deg = None
            elevation_deg = None
            if detections_msg is not None:
                for detection in detections_msg.detections:
                    for result in detection.results:
                        if result.hypothesis.class_id == self.vision_land_target_class_id:
                            azimuth_deg = result.pose.pose.position.x
                            elevation_deg = result.pose.pose.position.y
                            break
                    if azimuth_deg is not None:
                        break

            if azimuth_deg is None:
                # Target not currently visible: hold position, don't descend blind.
                east_vel = 0.0
                north_vel = 0.0
                vertical_vel = 0.0
                self.vision_land_centered_since = None
            else:
                east_vel = max(-self.vision_land_max_horizontal_vel, min(
                    self.vision_land_max_horizontal_vel, self.vision_land_kp_horizontal * azimuth_deg))
                north_vel = max(-self.vision_land_max_horizontal_vel, min(
                    self.vision_land_max_horizontal_vel, self.vision_land_kp_horizontal * elevation_deg))
                centered = (abs(azimuth_deg) < self.vision_land_centered_threshold_deg
                            and abs(elevation_deg) < self.vision_land_centered_threshold_deg)
                # Only descend once centered — avoids diving toward a target seen at an angle.
                vertical_vel = -self.vision_land_descend_vel if centered else 0.0
                low_enough = current_alt is not None and current_alt <= self.vision_land_land_alt_msl
                now = self.get_clock().now()
                if centered and low_enough:
                    if self.vision_land_centered_since is None:
                        self.vision_land_centered_since = now
                    elif (now - self.vision_land_centered_since).nanoseconds / 1e9 >= self.vision_land_min_confirm_secs:
                        self.get_logger().info(
                            f"vision_land: centered and low ({current_alt:.1f} MSL) for "
                            f"{self.vision_land_min_confirm_secs}s, advancing to land.")
                        self.vision_land_active = False
                        self.mission_step += 1
                        return
                else:
                    self.vision_land_centered_since = None

            last_send_elapsed = (999.0 if self.vision_land_last_send is None else
                (self.get_clock().now() - self.vision_land_last_send).nanoseconds / 1e9)
            if last_send_elapsed >= 1.0:
                req = SetReposition.Request()
                req.east = east_vel
                req.north = north_vel
                req.altitude = 15.0  # > 10 → GPS-free velocity mode
                req.vertical_velocity = vertical_vel
                self._call_service_no_advance(self._reposition_client, req)
                self.vision_land_last_send = self.get_clock().now()
            return

        if self.reach_position_active:
            with self.data_lock:
                lat = self.lat
                home_lat = self.home_lat
            if lat is not None and home_lat is not None:
                current_north = (lat - home_lat) * 111320.0
                dist_to_target = abs(current_north - self.reach_position_target_north)
                if dist_to_target < self.reach_position_threshold:
                    self.get_logger().info(
                        f"wait_to_reach_position: arrived — north≈{current_north:.1f}m "
                        f"(target={self.reach_position_target_north:.1f}m, err={dist_to_target:.1f}m).")
                    self.reach_position_active = False
                    self.mission_step += 1
                    return
            elapsed = (self.get_clock().now() - self.reach_position_start).nanoseconds / 1e9
            if elapsed > self.reach_position_timeout:
                self.get_logger().error("Timeout in wait_to_reach_position.")
                self.reach_position_active = False
                self.mission_step = -1
            return

        # If a "Wait" is active, check time: if still waiting, return. If done, clear wait and proceed
        if self.wait_start_time is not None:
            elapsed = (self.get_clock().now() - self.wait_start_time).nanoseconds / 1e9
            if elapsed < self.current_wait_duration:
                return # Still waiting
            else:
                self.get_logger().info(f"Wait complete")
                self.wait_start_time = None # Clear the wait flag
                self.mission_step += 1 # Advance to next step

        # End the mission
        if (self.mission_step >= len(self.mission_plan)) or (self.mission_step == -1):
            if self.mission_step == -1:
                self.get_logger().info("Mission Failed")
            else:
                self.get_logger().info("Mission Complete")
            self.conops_timer.cancel()
            rclpy.shutdown()
            return

        # Avoid executing the same step more than once
        if self.mission_step == self.last_executed_step:
            return

        # Continue the mission
        step = self.mission_plan[self.mission_step]
        action_type = step['action']
        params = step.get('params', {})
        self.get_logger().info(f"Executing step {self.mission_step}: {action_type}")

        if action_type == 'wait':
            self.last_executed_step = self.mission_step
            self.wait_start_time = self.get_clock().now()
            self.current_wait_duration = float(params.get('duration', 0.0))
            return

        elif action_type == 'takeoff':
            goal = Takeoff.Goal()
            goal.takeoff_altitude = float(params.get('takeoff_altitude', 20.0))
            goal.vtol_transition_heading = float(params.get('vtol_transition_heading', 0.0))
            goal.vtol_loiter_nord = float(params.get('vtol_loiter_nord', 100.0))
            goal.vtol_loiter_east = float(params.get('vtol_loiter_east', 100.0))
            goal.vtol_loiter_alt = float(params.get('vtol_loiter_alt', 120.0))
            if not self.send_goal(self._takeoff_client, goal):
                return

        elif action_type == 'land':
            goal = Land.Goal()
            goal.landing_altitude = float(params.get('landing_altitude', 20.0))
            goal.vtol_transition_heading = float(params.get('vtol_transition_heading', 0.0))
            if not self.send_goal(self._land_client, goal):
                return

        elif action_type == 'orbit':
            goal = Orbit.Goal()
            goal.east = float(params.get('east', 0.0))
            goal.north = float(params.get('north', 0.0))
            goal.altitude = float(params.get('altitude', 20.0))
            goal.radius = float(params.get('radius', 10.0))
            if not self.send_goal(self._orbit_client, goal):
                return

        elif action_type == 'offboard':
            if (os.getenv('AUTOPILOT', '') == 'ardupilot') and (os.getenv('DRONE_TYPE', '') != 'quad'):
                self.get_logger().warn("Offboard action is not supported by Ardupilot VTOL. Skip.")
                self.mission_step += 1
                return
            default_setpoint_type = 2 if os.getenv('AUTOPILOT', '') == 'px4' else 3 # 2: PX4 trajectory reference, 3: ArduPilot velocity
            goal = Offboard.Goal()
            goal.offboard_setpoint_type = int(params.get('offboard_setpoint_type', default_setpoint_type))
            goal.max_duration_sec = float(params.get('max_duration_sec', 10.0))
            if not self.send_goal(self._offboard_client, goal):
                return

        elif action_type in ('reposition', 'go_to_known_gps_waypoint'):
            if os.getenv('DRONE_TYPE', '') != 'quad':
                self.get_logger().warn("Reposition action is only supported for 'quad' drone type. Skip.")
                self.mission_step += 1
                return
            req = SetReposition.Request()
            req.east = float(params.get('east', 0.0))
            req.north = float(params.get('north', 0.0))
            req.altitude = float(params.get('altitude', 50.0))
            self.call_service(self._reposition_client, req)
            
        elif action_type == 'speed':
            req = SetSpeed.Request()
            req.speed = float(params.get('speed', 15.0))
            self.call_service(self._speed_client, req)

        elif action_type == 'move_rov':
            x = float(params.get('x', 25.0))
            y = float(params.get('y', 0.0))
            z = float(params.get('z', -0.5))
            threshold = float(params.get('threshold', 1.5))
            timeout = float(params.get('timeout', 120.0))
            cmd = Point()
            cmd.x = x
            cmd.y = y
            cmd.z = z
            self._rov_cmd_pub.publish(cmd)
            self.get_logger().info(f"ROV waypoint → ({x:.1f}, {y:.1f}, {z:.1f}), threshold={threshold}m")
            with self.data_lock:
                self.rov_waypoint_target = (x, y, z)
                self.rov_waypoint_threshold = threshold
                self.rov_waypoint_timeout = timeout
                self.rov_waypoint_start = self.get_clock().now()
                self.rov_waypoint_waiting = True

        elif action_type == 'wait_for_bool_topic':
            topic = str(params.get('topic', '/bluerov0/tour_done'))
            timeout = float(params.get('timeout', 300.0))
            with self.data_lock:
                flag = self.bool_topic_wait_flag
            if flag:
                self.get_logger().info(f"Bool topic '{topic}' already True.")
                self.bool_topic_wait_flag = False
                self.mission_step += 1
                return
            self.get_logger().info(f"Waiting for '{topic}' (timeout={timeout}s)...")
            self.bool_topic_wait_topic = topic
            self.bool_topic_wait_start = self.get_clock().now()
            self.bool_topic_wait_timeout = timeout

        elif action_type == 'descend_to_water':
            timeout = float(params.get('timeout', 180.0))
            req = SetReposition.Request()
            req.north = 0.0
            req.east = 0.0
            req.altitude = -10.0  # altitude < -1 → vel.z=-2 m/s, GPS-free descent
            # Use no-advance: mission_step is advanced by nav_mode_callback when AQUATIC_NAV fires,
            # not by service_response_callback (which would cause a double-increment).
            self._call_service_no_advance(self._reposition_client, req)
            self.descend_to_water_active = True
            self.descend_to_water_start = self.get_clock().now()
            self.descend_to_water_last_send = self.get_clock().now()
            self.descend_to_water_timeout = timeout
            self.get_logger().info(f"descend_to_water: descending at 2 m/s until AQUATIC_NAV (timeout={timeout}s).")

        elif action_type == 'ascend_from_water':
            timeout = float(params.get('timeout', 120.0))
            req = SetReposition.Request()
            req.north = 0.0
            req.east = 0.0
            req.altitude = 15.0  # > 10 → vel.z = +2 m/s, GPS-free ascent
            req.vertical_velocity = 2.0
            self._call_service_no_advance(self._reposition_client, req)
            self.ascend_from_water_active = True
            self.ascend_from_water_start = self.get_clock().now()
            self.ascend_from_water_timeout = timeout
            self.ascend_from_water_last_send = self.get_clock().now()
            self.get_logger().info(f"ascend_from_water: ascending at 2 m/s until AERIAL_NAV (timeout={timeout}s).")

        elif action_type == 'ascend_to_altitude':
            target_msl = float(params.get('altitude_msl', 575.0))
            timeout = float(params.get('timeout', 120.0))
            with self.data_lock:
                current_alt = self.alt_msl
            if current_alt is not None and current_alt >= target_msl:
                self.get_logger().info(f"ascend_to_altitude: already at {current_alt:.1f} MSL.")
                self.mission_step += 1
                return
            req = SetReposition.Request()
            req.north = 0.0
            req.east = 0.0
            req.altitude = 15.0
            req.vertical_velocity = 2.0
            self._call_service_no_advance(self._reposition_client, req)
            self.ascend_to_alt_active = True
            self.ascend_to_alt_target_msl = target_msl
            self.ascend_to_alt_start = self.get_clock().now()
            self.ascend_to_alt_timeout = timeout
            self.ascend_to_alt_last_send = self.get_clock().now()
            alt_str = f"{current_alt:.1f}" if current_alt is not None else "N/A"
            self.get_logger().info(
                f"ascend_to_altitude: ascending to {target_msl:.1f} MSL (current {alt_str}).")

        elif action_type == 'vision_land':
            self.vision_land_target_class_id = str(params.get('target_class_id', 'shape_target'))
            self.vision_land_timeout = float(params.get('timeout', 120.0))
            self.vision_land_centered_threshold_deg = float(params.get('centered_threshold_deg', 5.0))
            self.vision_land_min_confirm_secs = float(params.get('min_confirm_secs', 2.0))
            self.vision_land_land_alt_msl = float(params.get('land_alt_msl', 571.5))
            self.vision_land_active = True
            self.vision_land_start = self.get_clock().now()
            self.vision_land_last_send = None
            self.vision_land_centered_since = None
            self.get_logger().info(
                f"vision_land: centering on '{self.vision_land_target_class_id}' "
                f"(timeout={self.vision_land_timeout}s).")
            return

        elif action_type == 'wait_to_reach_position':
            target_north = float(params.get('north', 0.0))
            threshold = float(params.get('threshold', 3.0))
            timeout = float(params.get('timeout', 60.0))
            with self.data_lock:
                lat = self.lat
                home_lat = self.home_lat
            if lat is not None and home_lat is not None:
                current_north = (lat - home_lat) * 111320.0
                if abs(current_north - target_north) < threshold:
                    self.get_logger().info(
                        f"wait_to_reach_position: already at north≈{current_north:.1f}m.")
                    self.mission_step += 1
                    return
            self.reach_position_active = True
            self.reach_position_target_north = target_north
            self.reach_position_threshold = threshold
            self.reach_position_start = self.get_clock().now()
            self.reach_position_timeout = timeout
            self.get_logger().info(
                f"wait_to_reach_position: waiting for north={target_north:.1f}m "
                f"(threshold={threshold:.1f}m, timeout={timeout:.0f}s).")

        elif action_type == 'wait_for_nav_mode':
            target_mode = str(params.get('mode', 'AERIAL_NAV'))
            timeout = float(params.get('timeout', 30.0))
            with self.data_lock:
                current_mode = self.current_nav_mode
            if current_mode == target_mode:
                self.get_logger().info(f"Nav mode already '{target_mode}'.")
                self.mission_step += 1
                return
            self.get_logger().info(f"Waiting for nav mode '{target_mode}' (timeout={timeout}s)...")
            self.nav_mode_wait_target = target_mode
            self.nav_mode_wait_start = self.get_clock().now()
            self.nav_mode_wait_timeout = timeout

        elif action_type == 'wait_for_land_complete':
            timeout = float(params.get('timeout', 60.0))
            with self.data_lock:
                ls = self.landed_state
                nav_mode = self.current_nav_mode
            if ls == 1:  # already on ground (platform or rigid surface)
                surface = 'water' if nav_mode == 'AQUATIC_NAV' else 'platform'
                self.landing_surface = surface
                self.get_logger().info(f"Land complete (already ON_GROUND). Surface: {surface}.")
                self.mission_step += 1
                return
            if nav_mode == 'AQUATIC_NAV':  # already floating on water surface
                self.landing_surface = 'water'
                self.get_logger().info("Land complete (already AQUATIC_NAV). Surface: water.")
                self.mission_step += 1
                return
            self.get_logger().info(f"Waiting for land_complete signal (timeout={timeout}s)...")
            self.land_complete_waiting = True
            self.land_complete_start = self.get_clock().now()
            self.land_complete_timeout = timeout
            self.land_alt_prev = None
            self.land_alt_stable_start = None

        else:
            self.get_logger().error(f"Unknown action: {action_type}")
            self.mission_step = -1
            return

        self.last_executed_step = self.mission_step

def main(args=None):
    parser = argparse.ArgumentParser(description="Mission Node.")
    parser.add_argument('--conops', type=str, default='yalla.yaml', help="Path to the mission YAML file")

    cli_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    mission_node = MissionNode(mission_file=cli_args.conops)

    executor = MultiThreadedExecutor() # Or set MultiThreadedExecutor(num_threads=4)
    executor.add_node(mission_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        mission_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
