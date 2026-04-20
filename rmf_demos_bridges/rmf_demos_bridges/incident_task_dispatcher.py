import argparse
import json
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rmf_task_msgs.msg import ApiRequest, DispatchStates
from std_msgs.msg import String


class IncidentTaskDispatcher(Node):
    def __init__(self, argv=sys.argv):
        parser = argparse.ArgumentParser()
        parser.add_argument('--classification-topic', default='/obstacle_classifications',
                            help='Classification topic with obstacle_type labels')
        parser.add_argument('--alert-topic', default='/rmf_demo_alerts',
                            help='Raw alert topic; used when alerts already contain semantic labels')
        parser.add_argument('--dispatch-states-topic', default='/dispatch_states',
                            help='Dispatch states topic to detect active clean tasks')
        parser.add_argument('--task-api-topic', default='/task_api_requests',
                            help='Task API request topic')
        parser.add_argument('--enabled-obstacle-types', default='puddle,water_puddle',
                            help='Comma-separated obstacle types that should create clean tasks')
        parser.add_argument('--clean-zone', default='clean_inno_room',
                            help='Fallback clean zone when no per-level mapping is available')
        parser.add_argument('--level-zone-map-json', default='',
                            help='Optional JSON map from level_name to clean zone')
        parser.add_argument('--min-confidence', type=float, default=0.6,
                            help='Minimum confidence required to auto-create a clean task')
        parser.add_argument('--cooldown-sec', type=float, default=180.0,
                            help='Debounce period for repeated incidents from the same key')
        parser.add_argument('--allow-while-clean-active', action='store_true',
                            help='Allow auto-dispatch even when a clean task is already active')
        parser.add_argument('--key-distance-bucket-m', type=float, default=0.5,
                            help='Bucket size for distance used in incident dedup keys')

        self.args, _ = parser.parse_known_args(argv[1:])
        super().__init__('incident_task_dispatcher')

        self._enabled_types = {
            t.strip().lower() for t in self.args.enabled_obstacle_types.split(',') if t.strip()
        }
        self._level_zone_map = {}
        if self.args.level_zone_map_json:
            try:
                parsed = json.loads(self.args.level_zone_map_json)
                if isinstance(parsed, dict):
                    self._level_zone_map = {str(k): str(v) for k, v in parsed.items()}
            except Exception as err:
                self.get_logger().warn(
                    f'Unable to parse --level-zone-map-json: {err}; using fallback zone only')

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)

        self._task_pub = self.create_publisher(ApiRequest, self.args.task_api_topic, transient_qos)
        self._classification_sub = self.create_subscription(
            String,
            self.args.classification_topic,
            self._classification_callback,
            10)
        self._alert_sub = self.create_subscription(
            String,
            self.args.alert_topic,
            self._alert_callback,
            10)
        self._dispatch_states_sub = self.create_subscription(
            DispatchStates,
            self.args.dispatch_states_topic,
            self._dispatch_states_callback,
            10)

        self._last_dispatch_time = {}
        self._active_clean_task_ids = set()

        self.get_logger().info(
            f'Incident dispatcher active: enabled_types={sorted(self._enabled_types)}, '
            f'clean_zone={self.args.clean_zone}, cooldown={self.args.cooldown_sec:.1f}s')

    def _dispatch_states_callback(self, msg: DispatchStates):
        active_clean_ids = set()
        for state in msg.active:
            if state.status not in (2, 3):
                continue
            task_id = state.task_id.strip()
            if not task_id:
                continue
            if task_id.startswith('clean.') or task_id.startswith('clean_'):
                active_clean_ids.add(task_id)
        self._active_clean_task_ids = active_clean_ids

    def _classification_callback(self, msg: String):
        payload = self._decode_json(msg.data)
        if payload is None:
            return
        self._handle_incident(payload, source_hint='classification')

    def _alert_callback(self, msg: String):
        payload = self._decode_json(msg.data)
        if payload is None:
            return
        self._handle_incident(payload, source_hint='alert')

    def _decode_json(self, text: str):
        if not text:
            return None
        try:
            data = json.loads(text)
            if isinstance(data, dict):
                return data
        except Exception:
            pass
        return None

    def _handle_incident(self, payload: dict, source_hint: str):
        obstacle_type = str(payload.get('obstacle_type', '')).strip().lower()
        if obstacle_type not in self._enabled_types:
            return

        confidence = float(payload.get('confidence', 1.0))
        if confidence < self.args.min_confidence:
            return

        if self._active_clean_task_ids and not self.args.allow_while_clean_active:
            return

        key = self._incident_key(payload, obstacle_type, source_hint)
        now = time.time()
        last = self._last_dispatch_time.get(key, 0.0)
        if now - last < max(1.0, self.args.cooldown_sec):
            return

        zone = self._zone_for_payload(payload)
        request_id = f'auto_clean_{uuid.uuid4()}'
        task_payload = self._build_clean_task_payload(zone)

        msg = ApiRequest()
        msg.request_id = request_id
        msg.json_msg = json.dumps(task_payload)
        self._task_pub.publish(msg)

        self._last_dispatch_time[key] = now
        self.get_logger().warn(
            f'Auto-dispatched clean task for {obstacle_type} in zone={zone} '
            f'(source={source_hint}, key={key})')

    def _incident_key(self, payload: dict, obstacle_type: str, source_hint: str) -> str:
        level_name = str(payload.get('level_name', '')).strip()
        obstacle_name = str(payload.get('obstacle_name', '')).strip()
        robot_name = str(payload.get('robot_name', '')).strip()
        sector = str(payload.get('sector', '')).strip()
        scan_topic = str(payload.get('scan_topic', '')).strip()

        distance = float(payload.get('distance_estimate', 0.0))
        bucket = max(0.1, self.args.key_distance_bucket_m)
        distance_bucket = int(distance / bucket)

        if obstacle_name:
            return f'{source_hint}|{obstacle_type}|{level_name}|{obstacle_name}'

        return (
            f'{source_hint}|{obstacle_type}|{level_name}|{robot_name}|'
            f'{scan_topic}|{sector}|d{distance_bucket}'
        )

    def _zone_for_payload(self, payload: dict) -> str:
        level_name = str(payload.get('level_name', '')).strip()
        if level_name and level_name in self._level_zone_map:
            return self._level_zone_map[level_name]
        return self.args.clean_zone

    def _build_clean_task_payload(self, zone: str) -> dict:
        now = self.get_clock().now().to_msg()
        earliest_start_ms = now.sec * 1000 + round(now.nanosec / 10**6)
        return {
            'type': 'dispatch_task_request',
            'request': {
                'unix_millis_earliest_start_time': earliest_start_ms,
                'category': 'clean',
                'description': {
                    'zone': zone,
                },
            },
        }


def main(argv=sys.argv):
    rclpy.init(args=argv)
    node = IncidentTaskDispatcher(argv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
