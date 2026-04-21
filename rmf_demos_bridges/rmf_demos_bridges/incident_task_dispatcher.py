import argparse
import json
import math
import shutil
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid

import rclpy
from rmf_fleet_msgs.msg import FleetState
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
        parser.add_argument('--cleaner-name-token', default='cleaner',
                    help='Case-insensitive token used to identify cleaning robots')
        parser.add_argument('--cleaner-dwell-sec', type=float, default=10.0,
                    help='Seconds to simulate cleaner work at puddle location')
        parser.add_argument('--entity-register-topic', default='/sim_injected_entities',
                    help='Topic used by perception_bridge to track active injected entities')
        parser.add_argument('--fleet-states-topic', default='/fleet_states',
                    help='FleetState topic used to track robot pose during puddle response')
        parser.add_argument('--cleaner-fleet-manager-prefix', default='http://127.0.0.1:22013',
                    help='Fleet manager base URL used for cleaner navigate/stop commands')
        parser.add_argument('--arrival-threshold-m', type=float, default=0.4,
                    help='Distance threshold to treat cleaner as arrived at puddle')
        parser.add_argument('--navigate-timeout-sec', type=float, default=60.0,
                    help='Maximum navigation time before forcing dwell phase')
        parser.add_argument('--resume-clean-after-local-response', action='store_true',
                    help='After local puddle handling, submit a clean task to resume workflow')
        parser.add_argument('--sim-world', default='sim_world',
                    help='Ignition world name used when removing puddle model')
        parser.add_argument('--remove-entity-on-clean', action='store_true',
                    help='Attempt to remove puddle model from simulation after clean completes')

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
        self._alert_pub = self.create_publisher(String, self.args.alert_topic, 10)
        self._entity_pub = self.create_publisher(String, self.args.entity_register_topic, 10)
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
        self._fleet_states_sub = self.create_subscription(
            FleetState,
            self.args.fleet_states_topic,
            self._fleet_states_callback,
            10)

        self._last_dispatch_time = {}
        self._active_clean_task_ids = set()
        self._active_clean_task_robot_keys = set()
        self._active_cleaning_work = {}
        self._robot_positions = {}
        self._next_cmd_id = {}
        self._cleaning_timer = self.create_timer(0.5, self._process_cleaning_work)

        self.get_logger().info(
            f'Incident dispatcher active: enabled_types={sorted(self._enabled_types)}, '
            f'clean_zone={self.args.clean_zone}, cooldown={self.args.cooldown_sec:.1f}s')

    def _dispatch_states_callback(self, msg: DispatchStates):
        active_clean_ids = set()
        active_clean_robot_keys = set()
        for state in msg.active:
            if state.status not in (2, 3):
                continue
            task_id = state.task_id.strip()
            if not task_id:
                continue
            if task_id.startswith('clean.') or task_id.startswith('clean_'):
                active_clean_ids.add(task_id)
                robot_name = state.assignment.expected_robot_name.strip()
                if robot_name:
                    active_clean_robot_keys.add(self._robot_key(robot_name))
        self._active_clean_task_ids = active_clean_ids
        self._active_clean_task_robot_keys = active_clean_robot_keys

    def _fleet_states_callback(self, msg: FleetState):
        for robot in msg.robots:
            self._robot_positions[robot.name] = {
                'x': float(robot.location.x),
                'y': float(robot.location.y),
                'level_name': str(robot.location.level_name),
                'stamp': time.time(),
            }

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
        source = str(payload.get('source', '')).strip().lower()
        if source == 'incident_task_dispatcher':
            return

        obstacle_type = str(payload.get('obstacle_type', '')).strip().lower()
        if obstacle_type not in self._enabled_types:
            return

        robot_name = str(payload.get('robot_name', '')).strip()
        if self._is_cleaner_robot(robot_name):
            if self._start_cleaner_workflow(payload, obstacle_type):
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

    def _is_cleaner_robot(self, robot_name: str) -> bool:
        token = str(self.args.cleaner_name_token).strip().lower()
        if not token:
            return False
        return token in robot_name.lower()

    def _start_cleaner_workflow(self, payload: dict, obstacle_type: str) -> bool:
        obstacle_name = str(payload.get('obstacle_name', '')).strip()
        level_name = str(payload.get('level_name', '')).strip()
        if not obstacle_name or not level_name:
            return False

        key = f'{obstacle_type}|{level_name}|{obstacle_name}'
        if key in self._active_cleaning_work:
            return True

        obstacle_position = payload.get('obstacle_position', {})
        if not isinstance(obstacle_position, dict):
            obstacle_position = {}
        x = float(obstacle_position.get('x', payload.get('x', 0.0)))
        y = float(obstacle_position.get('y', payload.get('y', 0.0)))

        robot_name = str(payload.get('robot_name', '')).strip()
        now = time.time()
        if not self._send_cleaner_to_puddle(robot_name, level_name, x, y):
            self.get_logger().warn(
                f'Cleaner navigate command failed for {robot_name}; '
                f'falling back to timed local workflow for {obstacle_name}')

        had_active_clean_task = self._robot_key(robot_name) in self._active_clean_task_robot_keys
        self._active_cleaning_work[key] = {
            'obstacle_type': obstacle_type,
            'obstacle_name': obstacle_name,
            'level_name': level_name,
            'x': x,
            'y': y,
            'robot_name': robot_name,
            'started_at': now,
            'navigate_deadline': now + max(5.0, self.args.navigate_timeout_sec),
            'dwell_until': 0.0,
            'had_active_clean_task': had_active_clean_task,
            'phase': 'navigating',
        }

        self._publish_status_alert(
            robot_name=robot_name,
            level_name=level_name,
            obstacle_type=obstacle_type,
            obstacle_name=obstacle_name,
            x=x,
            y=y,
            action='Work Order: Diverting cleaner to puddle')

        self.get_logger().info(
            f'Cleaner workflow started for {obstacle_type} {obstacle_name} at '
            f'({x:.3f}, {y:.3f}) on {level_name}; phase=navigating')
        return True

    def _process_cleaning_work(self):
        if not self._active_cleaning_work:
            return

        now = time.time()
        completed_keys = []
        for key, state in self._active_cleaning_work.items():
            phase = str(state.get('phase', ''))
            if phase == 'navigating':
                arrived = self._is_robot_near_target(
                    robot_name=str(state['robot_name']),
                    level_name=str(state['level_name']),
                    x=float(state['x']),
                    y=float(state['y']))
                timed_out = now >= float(state.get('navigate_deadline', 0.0))
                if not arrived and not timed_out:
                    continue

                if timed_out and not arrived:
                    self.get_logger().warn(
                        f"Cleaner {state['robot_name']} did not reach puddle in time; "
                        'starting dwell anyway')

                state['phase'] = 'dwell'
                state['dwell_until'] = now + max(1.0, self.args.cleaner_dwell_sec)
                self._publish_status_alert(
                    robot_name=str(state['robot_name']),
                    level_name=str(state['level_name']),
                    obstacle_type=str(state['obstacle_type']),
                    obstacle_name=str(state['obstacle_name']),
                    x=float(state['x']),
                    y=float(state['y']),
                    action='Work Order: Cleaning in progress')
                continue

            if phase == 'dwell' and now >= float(state.get('dwell_until', 0.0)):
                completed_keys.append(key)

        for key in completed_keys:
            state = self._active_cleaning_work.pop(key, None)
            if not state:
                continue

            self._publish_status_alert(
                robot_name=str(state['robot_name']),
                level_name=str(state['level_name']),
                obstacle_type=str(state['obstacle_type']),
                obstacle_name=str(state['obstacle_name']),
                x=float(state['x']),
                y=float(state['y']),
                action='Work Order: Cleaned')
            self._deactivate_entity(state)

            if self.args.remove_entity_on_clean:
                self._remove_entity_from_sim(str(state['obstacle_name']))

            if self.args.resume_clean_after_local_response and bool(state.get('had_active_clean_task', False)):
                zone = self._level_zone_map.get(str(state['level_name']), self.args.clean_zone)
                self._dispatch_resume_clean_task(zone, state)

    def _robot_key(self, robot_name: str) -> str:
        return str(robot_name).strip().lower()

    def _is_robot_near_target(self, *, robot_name: str, level_name: str, x: float, y: float) -> bool:
        pose = self._robot_positions.get(robot_name)
        if pose is None:
            return False

        pose_level = str(pose.get('level_name', ''))
        if pose_level and level_name and pose_level != level_name:
            return False

        dx = float(pose.get('x', 0.0)) - x
        dy = float(pose.get('y', 0.0)) - y
        distance = math.hypot(dx, dy)
        return distance <= max(0.1, self.args.arrival_threshold_m)

    def _next_command_id(self, robot_name: str) -> int:
        key = self._robot_key(robot_name)
        next_id = int(self._next_cmd_id.get(key, 1000)) + 1
        self._next_cmd_id[key] = next_id
        return next_id

    def _send_cleaner_to_puddle(self, robot_name: str, level_name: str, x: float, y: float) -> bool:
        if not robot_name:
            return False

        cmd_id = self._next_command_id(robot_name)
        stop_path = (
            '/open-rmf/rmf_demos_fm/stop_robot?'
            f'robot_name={urllib.parse.quote(robot_name)}&cmd_id={cmd_id}'
        )
        self._fleet_get(stop_path)

        cmd_id = self._next_command_id(robot_name)
        navigate_path = (
            '/open-rmf/rmf_demos_fm/navigate/?'
            f'robot_name={urllib.parse.quote(robot_name)}&cmd_id={cmd_id}'
        )
        body = {
            'map_name': level_name,
            'destination': {
                'x': float(x),
                'y': float(y),
                'yaw': 0.0,
            },
            'speed_limit': 0.0,
        }
        ok = self._fleet_post(navigate_path, body)
        if ok:
            self.get_logger().info(
                f'Navigate accepted for {robot_name} -> ({x:.3f}, {y:.3f}) on {level_name}')
        return ok

    def _fleet_post(self, path: str, body: dict) -> bool:
        url = self.args.cleaner_fleet_manager_prefix.rstrip('/') + path
        data = json.dumps(body).encode('utf-8')
        req = urllib.request.Request(
            url,
            data=data,
            headers={'Content-Type': 'application/json'},
            method='POST')
        try:
            with urllib.request.urlopen(req, timeout=4.0) as resp:
                text = resp.read().decode('utf-8').strip()
            payload = json.loads(text) if text else {}
            return bool(payload.get('success', True))
        except (urllib.error.URLError, ValueError, TimeoutError) as err:
            self.get_logger().warn(f'Fleet POST failed for {url}: {err}')
            return False

    def _fleet_get(self, path: str) -> bool:
        url = self.args.cleaner_fleet_manager_prefix.rstrip('/') + path
        req = urllib.request.Request(url, method='GET')
        try:
            with urllib.request.urlopen(req, timeout=4.0) as resp:
                text = resp.read().decode('utf-8').strip()
            payload = json.loads(text) if text else {}
            return bool(payload.get('success', True))
        except (urllib.error.URLError, ValueError, TimeoutError) as err:
            self.get_logger().warn(f'Fleet GET failed for {url}: {err}')
            return False

    def _dispatch_resume_clean_task(self, zone: str, state: dict):
        request_id = f'resume_clean_{uuid.uuid4()}'
        task_payload = self._build_clean_task_payload(zone)
        msg = ApiRequest()
        msg.request_id = request_id
        msg.json_msg = json.dumps(task_payload)
        self._task_pub.publish(msg)
        self.get_logger().info(
            f"Submitted best-effort resume clean task after puddle {state['obstacle_name']} in zone={zone}")

    def _publish_status_alert(self, *, robot_name: str, level_name: str,
                              obstacle_type: str, obstacle_name: str,
                              x: float, y: float, action: str):
        payload = {
            'timestamp': time.time(),
            'robot_name': robot_name,
            'level_name': level_name,
            'obstacle_type': obstacle_type,
            'obstacle_name': obstacle_name,
            'distance_estimate': 0.0,
            'obstacle_position': {
                'x': round(float(x), 3),
                'y': round(float(y), 3),
            },
            'recommended_action': action,
            'source': 'incident_task_dispatcher',
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._alert_pub.publish(msg)

    def _deactivate_entity(self, state: dict):
        payload = {
            'entities': [{
                'name': str(state['obstacle_name']),
                'classification': str(state['obstacle_type']),
                'level_name': str(state['level_name']),
                'x': float(state['x']),
                'y': float(state['y']),
                'active': False,
            }]
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._entity_pub.publish(msg)
        self.get_logger().info(
            f"Marked entity {state['obstacle_name']} inactive on "
            f"{self.args.entity_register_topic}")

    def _remove_entity_from_sim(self, entity_name: str):
        ign = shutil.which('ign')
        if ign is None:
            self.get_logger().warn('ign command not found; cannot remove obstacle entity from simulation')
            return

        service = f"/world/{self.args.sim_world}/remove"
        request = f'name: "{entity_name}" type: MODEL'
        command = [
            ign, 'service',
            '-s', service,
            '--reqtype', 'ignition.msgs.Entity',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '3000',
            '--req', request,
        ]
        try:
            completed = subprocess.run(command, check=True, capture_output=True, text=True)
            if completed.stdout.strip():
                self.get_logger().info(completed.stdout.strip())
            self.get_logger().info(f'Removed entity from sim: {entity_name}')
        except subprocess.CalledProcessError as err:
            stderr = err.stderr.strip() if err.stderr else str(err)
            self.get_logger().warn(f'Failed to remove entity {entity_name}: {stderr}')

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
