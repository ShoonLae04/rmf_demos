import argparse
import json
import math
import re
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import Odometry


class ScanObstacleDetector(Node):
    def __init__(self, argv=sys.argv):
        parser = argparse.ArgumentParser()
        parser.add_argument('-s', '--scan-topic', default='/scan',
                            help='LaserScan topic to monitor')
        parser.add_argument('--scan-topic-regex', default=r'.*scan$',
                            help='Regex used for auto-discovered LaserScan topics')
        parser.add_argument('--auto-discover-scans', dest='auto_discover_scans',
                            action='store_true',
                            help='Automatically subscribe to LaserScan topics, including robots spawned later')
        parser.add_argument('--no-auto-discover-scans', dest='auto_discover_scans',
                            action='store_false',
                            help='Disable automatic LaserScan topic discovery')
        parser.set_defaults(auto_discover_scans=True)
        parser.add_argument('--discovery-period-sec', type=float, default=2.0,
                            help='Seconds between scan topic discovery checks')
        parser.add_argument('-a', '--alert-topic', default='/rmf_demo_alerts',
                            help='Alert topic to publish')
        parser.add_argument('-r', '--range-threshold', type=float, default=1.5,
                            help='Distance threshold (m) for obstacle alerts')
        parser.add_argument('-w', '--window-deg', type=float, default=30.0,
                            help='Half-window angle in degrees around scan center')
        parser.add_argument('-c', '--cooldown-sec', type=float, default=1.5,
                            help='Minimum seconds between repeated alerts')
        parser.add_argument('--min-consecutive-hits', type=int, default=2,
                            help='Minimum consecutive below-threshold scans before alerting')
        parser.add_argument('--require-motion', action='store_true', default=False,
                            help='Only alert when robot is in motion (velocity > threshold)')
        parser.add_argument('--motion-velocity-threshold', type=float, default=0.05,
                            help='Velocity threshold (m/s) to consider robot as moving')
        parser.add_argument('--classification-topic', default='/obstacle_classifications',
                    help='Topic carrying obstacle classification JSON per robot')
        parser.add_argument('--classification-ttl-sec', type=float, default=3.0,
                    help='Seconds to keep latest obstacle classification for each robot')
        parser.add_argument('--emit-static-structure-label', dest='emit_static_structure_label',
                    action='store_true',
                    help='Tag persistent scan returns as static structures')
        parser.add_argument('--no-emit-static-structure-label', dest='emit_static_structure_label',
                    action='store_false',
                    help='Do not tag persistent scan returns as static structures')
        parser.set_defaults(emit_static_structure_label=True)
        parser.add_argument('--static-hit-threshold', type=int, default=6,
                    help='Consecutive stable hits needed to label static structures')
        parser.add_argument('--static-distance-epsilon', type=float, default=0.15,
                    help='Max range delta (m) between hits to consider them static')
        parser.add_argument('--static-obstacle-type', default='static_structure',
                    help='Obstacle type used for persistent static returns')
        parser.add_argument('--robot-name', default='',
                            help='Optional robot name to include in alerts')

        self.args, _ = parser.parse_known_args(argv[1:])
        super().__init__('scan_obstacle_detector')

        # QoS profiles for sensor data (best_effort, volatile)
        self._sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)
        # QoS profile for odometry (best_effort, volatile)
        self._motion_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)

        self._alert_pub = self.create_publisher(String, self.args.alert_topic, 10)
        self._scan_subscriptions = {}
        self._consecutive_hits = {}
        self._static_hits = {}
        self._last_alert_times = {}
        self._robot_velocities = {}
        self._obstacle_labels = {}
        self._missing_motion_logged = set()

        self._classification_sub = self.create_subscription(
            String,
            self.args.classification_topic,
            self._classification_callback,
            10)

        self._topic_regex = None
        try:
            self._topic_regex = re.compile(self.args.scan_topic_regex)
        except re.error as err:
            self.get_logger().warn(
                f'Invalid --scan-topic-regex "{self.args.scan_topic_regex}": {err}. '
                'Falling back to .*scan$')
            self._topic_regex = re.compile(r'.*scan$')

        self._subscribe_to_scan_topic(self.args.scan_topic)
        if self.args.auto_discover_scans:
            self._discovery_timer = self.create_timer(
                max(0.5, self.args.discovery_period_sec),
                self._discover_scan_topics)
            self._discover_scan_topics()
        else:
            self._discovery_timer = None

        if self.args.require_motion:
            self._odom_subscriptions = {}
            self._motion_discovery_timer = self.create_timer(
                max(0.5, self.args.discovery_period_sec),
                self._discover_odometry_topics)
            self._discover_odometry_topics()
        else:
            self._motion_discovery_timer = None

        self._last_scan_time = 0.0
        self._health_timer = self.create_timer(5.0, self._health_check)

        self.get_logger().info(
            f'Scan detector active. seed_topic={self.args.scan_topic}, '
            f'auto_discover={self.args.auto_discover_scans}, '
            f'min_consecutive_hits={self.args.min_consecutive_hits}, '
            f'require_motion={self.args.require_motion}, '
            f'alerting under {self.args.range_threshold:.2f}m')

    def _classification_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return

        robot_name = payload.get('robot_name', '')
        obstacle_type = payload.get('obstacle_type', '')
        if not robot_name or not obstacle_type:
            return

        confidence = float(payload.get('confidence', 1.0))
        self._obstacle_labels[self._normalized_robot_name(robot_name)] = {
            'obstacle_type': obstacle_type,
            'confidence': confidence,
            'timestamp': time.time(),
        }

    def _discover_odometry_topics(self):
        topic_names_and_types = self.get_topic_names_and_types(no_demangle=True)
        for topic_name, topic_types in topic_names_and_types:
            if 'nav_msgs/msg/Odometry' not in topic_types:
                continue
            robot_name = self._resolve_robot_name_from_topic(topic_name)
            if not robot_name:
                continue
            odom_key = robot_name
            if odom_key not in self._odom_subscriptions:
                self._odom_subscriptions[odom_key] = self.create_subscription(
                    Odometry,
                    topic_name,
                    lambda msg, rn=robot_name: self._odom_callback(msg, rn),
                    self._motion_qos)
                self.get_logger().info(f'Subscribed to odometry for robot {robot_name}: {topic_name}')

    def _odom_callback(self, msg: Odometry, robot_name: str):
        try:
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            vz = msg.twist.twist.linear.z
            speed = math.sqrt(vx*vx + vy*vy + vz*vz)
            self._robot_velocities[robot_name] = speed
            self._robot_velocities[robot_name.lower()] = speed
        except Exception:
            pass

    def _normalized_robot_name(self, robot_name: str) -> str:
        return robot_name.strip().lower() if robot_name else ''

    def _robot_speed(self, robot_name: str) -> float:
        if not robot_name:
            return 0.0
        direct = self._robot_velocities.get(robot_name)
        if direct is not None:
            return direct
        return self._robot_velocities.get(self._normalized_robot_name(robot_name), 0.0)

    def _classification_for_robot(self, robot_name: str):
        key = self._normalized_robot_name(robot_name)
        if not key:
            return None

        label = self._obstacle_labels.get(key)
        if not label:
            return None

        if time.time() - label['timestamp'] > max(0.1, self.args.classification_ttl_sec):
            return None

        return label

    def _resolve_robot_name_from_topic(self, topic_name: str) -> str:
        # Extract robot name from odometry topic paths like /TinyRobot1/odom or /robot/TinyRobot1/odom
        parts = [p for p in topic_name.split('/') if p]
        if not parts:
            return ''
        # Common patterns: first part or second-to-last part
        candidates = [parts[0]]
        if len(parts) > 1:
            candidates.append(parts[-2])
        for cand in candidates:
            if any(c.isdigit() for c in cand):  # Likely a robot name
                return cand
        return parts[0] if parts else ''

    def _subscribe_to_scan_topic(self, topic_name: str):
        topic_name = topic_name.strip()
        if not topic_name or topic_name in self._scan_subscriptions:
            return

        self._scan_subscriptions[topic_name] = self.create_subscription(
            LaserScan,
            topic_name,
            lambda msg, topic=topic_name: self._scan_callback(msg, topic),
            self._sensor_qos)
        self.get_logger().info(f'Subscribed to LaserScan topic: {topic_name}')

    def _discover_scan_topics(self):
        topic_names_and_types = self.get_topic_names_and_types(no_demangle=True)
        for topic_name, topic_types in topic_names_and_types:
            if 'sensor_msgs/msg/LaserScan' not in topic_types:
                continue
            if self._topic_regex and not self._topic_regex.search(topic_name):
                continue
            self._subscribe_to_scan_topic(topic_name)

    def _resolve_robot_name(self, frame_id: str, scan_topic: str) -> str:
        if self.args.robot_name:
            return self.args.robot_name

        if frame_id:
            # Typical frame_id: TinyRobot1/base_footprint/front_laser
            root = frame_id.split('/', 1)[0].strip()
            if root:
                return root

        topic_parts = [part for part in scan_topic.split('/') if part]
        if len(topic_parts) >= 2:
            return topic_parts[0]

        return ''

    def _classify_sector(self, angle_rad: float) -> str:
        angle_deg = math.degrees(angle_rad)
        if -20.0 <= angle_deg <= 20.0:
            return 'front'
        if angle_deg > 20.0:
            return 'left'
        return 'right'

    def _scan_callback(self, msg: LaserScan, scan_topic: str):
        self._last_scan_time = time.time()
        ranges = msg.ranges
        if not ranges:
            return

        center_index = len(ranges) // 2
        # Convert half-window angle into index span.
        angle_increment = msg.angle_increment if msg.angle_increment > 0.0 else 0.0
        if angle_increment <= 0.0:
            span = min(30, center_index)
        else:
            span = int(math.radians(max(self.args.window_deg, 1.0)) / angle_increment)
            span = max(1, min(span, center_index))

        start = max(0, center_index - span)
        end = min(len(ranges), center_index + span + 1)

        valid = []
        for value in ranges[start:end]:
            if not math.isfinite(value):
                continue
            if value <= 0.0:
                continue
            if msg.range_min > 0.0 and value < msg.range_min:
                continue
            if msg.range_max > 0.0 and value > msg.range_max:
                continue
            valid.append(value)

        if not valid:
            return

        min_distance = min(valid)
        min_index = None
        for index in range(start, end):
            value = ranges[index]
            if not math.isfinite(value):
                continue
            if value <= 0.0:
                continue
            if msg.range_min > 0.0 and value < msg.range_min:
                continue
            if msg.range_max > 0.0 and value > msg.range_max:
                continue
            if abs(value - min_distance) < 1e-6:
                min_index = index
                break

        if min_index is None:
            min_index = center_index

        bearing_rad = msg.angle_min + (min_index * msg.angle_increment)
        bearing_deg = math.degrees(bearing_rad)
        sector = self._classify_sector(bearing_rad)
        point_x = round(min_distance * math.cos(bearing_rad), 3)
        point_y = round(min_distance * math.sin(bearing_rad), 3)
        robot_name = self._resolve_robot_name(msg.header.frame_id, scan_topic)
        hit_key = (robot_name, scan_topic)

        if min_distance >= self.args.range_threshold:
            self._consecutive_hits[hit_key] = 0
            return

        hit_count = self._consecutive_hits.get(hit_key, 0) + 1
        self._consecutive_hits[hit_key] = hit_count
        if hit_count < max(1, self.args.min_consecutive_hits):
            return

        if self.args.require_motion:
            speed = self._robot_speed(robot_name)
            if speed < self.args.motion_velocity_threshold:
                robot_key = self._normalized_robot_name(robot_name)
                if robot_key and robot_key not in self._missing_motion_logged:
                    self.get_logger().info(
                        f'Waiting for motion on {robot_name}: '
                        f'speed={speed:.3f} < threshold={self.args.motion_velocity_threshold:.3f}')
                    self._missing_motion_logged.add(robot_key)
                return
            self._missing_motion_logged.discard(self._normalized_robot_name(robot_name))

        now = time.time()
        alert_key = (self._normalized_robot_name(robot_name), scan_topic)
        last_alert_time = self._last_alert_times.get(alert_key, 0.0)
        if now - last_alert_time < self.args.cooldown_sec:
            return

        self._last_alert_times[alert_key] = now
        classification = self._classification_for_robot(robot_name)
        obstacle_type = 'unknown_obstacle'
        if classification:
            obstacle_type = classification['obstacle_type']
        else:
            static_key = (self._normalized_robot_name(robot_name), scan_topic, sector)
            static_state = self._static_hits.get(static_key)
            if static_state is None:
                static_state = {'distance': min_distance, 'count': 1}
            else:
                if abs(min_distance - static_state['distance']) <= self.args.static_distance_epsilon:
                    static_state['count'] += 1
                else:
                    static_state['count'] = 1
                    static_state['distance'] = min_distance
            self._static_hits[static_key] = static_state

            if self.args.emit_static_structure_label and \
                    static_state['count'] >= max(2, self.args.static_hit_threshold):
                obstacle_type = self.args.static_obstacle_type

        payload = {
            'timestamp': now,
            'source': 'scan_obstacle_detector',
            'robot_name': robot_name,
            'scan_topic': scan_topic,
            'frame_id': msg.header.frame_id,
            'obstacle_type': obstacle_type,
            'distance_estimate': round(min_distance, 3),
            'bearing_rad': round(bearing_rad, 3),
            'bearing_deg': round(bearing_deg, 1),
            'sector': sector,
            'point_in_scan_frame': {'x': point_x, 'y': point_y},
            'scan_index': int(min_index),
            'threshold': self.args.range_threshold,
            'consecutive_hits': hit_count,
            'min_consecutive_hits': max(1, self.args.min_consecutive_hits),
            'recommended_action': 'Slow down, stop, and inspect nearby area',
        }

        if classification:
            payload['obstacle_confidence'] = round(classification['confidence'], 3)
            payload['classification_source'] = self.args.classification_topic

        if self.args.require_motion:
            payload['robot_speed'] = round(self._robot_speed(robot_name), 3)

        self.get_logger().warn(
            f"Laser obstacle detected at {min_distance:.2f}m on {scan_topic} "
            f"(robot={robot_name}, sector={sector}, bearing={bearing_deg:.1f}deg, "
            f"point=({point_x:.2f},{point_y:.2f}), hits={hit_count})")
        self._alert_pub.publish(String(data=json.dumps(payload)))

    def _health_check(self):
        if self._last_scan_time > 0.0:
            return

        monitored_topics = sorted(self._scan_subscriptions.keys())
        topic_publishers = [
            f'{topic}={self.count_publishers(topic)}'
            for topic in monitored_topics
        ]
        self.get_logger().warn(
            'No LaserScan data received yet '
            f'(monitored: {", ".join(monitored_topics) or "none"}; '
            f'publishers: {", ".join(topic_publishers) or "none"})')


def main(argv=sys.argv):
    rclpy.init(args=argv)
    node = ScanObstacleDetector(argv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
