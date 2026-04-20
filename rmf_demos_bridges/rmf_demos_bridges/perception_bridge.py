import argparse
import json
import math
import sys
import time
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node

from rmf_fleet_msgs.msg import RobotMode, RobotState
from std_msgs.msg import String


BLOCKING_MODES = {
    RobotMode.MODE_WAITING,
    RobotMode.MODE_ADAPTER_ERROR,
}


class PerceptionBridge(Node):
    def __init__(self, argv=sys.argv):
        parser = argparse.ArgumentParser()
        parser.add_argument('-r', '--robot_state_topic',
                            default='/robot_state',
                            help='Robot state topic to monitor')
        parser.add_argument('-e', '--entities_topic',
                            default='/sim_injected_entities',
                            help='Injected entity metadata topic')
        parser.add_argument('-a', '--alert_topic',
                            default='/rmf_demo_alerts',
                            help='Alert topic to publish')
        parser.add_argument('-d', '--alert_distance_threshold',
                            type=float,
                            default=1.0,
                            help='Distance threshold for alerting')
        parser.add_argument('-c', '--alert_cooldown_seconds',
                            type=float,
                            default=2.0,
                            help='Debounce interval for repeated alerts')
        parser.add_argument('--trigger-on-any-nearby', action='store_true',
                    help='Emit alerts for nearby injected entities even when '
                     'the robot is not in a blocking mode. Useful for '
                     'manual validation.')

        self.args, _ = parser.parse_known_args(argv[1:])
        super().__init__('perception_bridge')

        self._robots: Dict[str, Dict[str, object]] = {}
        self._entities: Dict[str, Dict[str, object]] = {}
        self._last_alert_time: Dict[Tuple[str, str, str], float] = {}

        self._alert_pub = self.create_publisher(String, self.args.alert_topic, 10)
        self._robot_state_sub = self.create_subscription(
            RobotState,
            self.args.robot_state_topic,
            self._robot_state_callback,
            10)
        self._entities_sub = self.create_subscription(
            String,
            self.args.entities_topic,
            self._entities_callback,
            10)

    def _robot_state_callback(self, msg: RobotState):
        self._robots[msg.name] = {
            'x': msg.location.x,
            'y': msg.location.y,
            'level_name': msg.location.level_name,
            'mode': msg.mode.mode,
            'mode_name': self._mode_name(msg.mode.mode),
        }
        self._evaluate_robot(msg.name)

    def _entities_callback(self, msg: String):
        entities = self._parse_entities_payload(msg.data)
        if not entities:
            return

        for entity in entities:
            self._entities[entity['name']] = entity

        for robot_name in list(self._robots.keys()):
            self._evaluate_robot(robot_name)

    def _parse_entities_payload(self, payload: str):
        if not payload:
            return []

        try:
            decoded = json.loads(payload)
        except Exception as exc:
            self.get_logger().warn(f'Unable to parse injected entity payload: {exc}')
            return []

        if isinstance(decoded, dict):
            if 'entities' in decoded:
                decoded = decoded['entities']
            else:
                decoded = [decoded]

        if not isinstance(decoded, list):
            self.get_logger().warn('Injected entity payload must be a dict or list')
            return []

        normalized = []
        for entry in decoded:
            entity = self._normalize_entity(entry)
            if entity is not None:
                normalized.append(entity)
        return normalized

    def _normalize_entity(self, entry: object):
        if not isinstance(entry, dict):
            return None

        name = str(entry.get('name', '')).strip()
        if not name:
            return None

        try:
            x = float(entry.get('x'))
            y = float(entry.get('y'))
        except (TypeError, ValueError):
            return None

        level_name = str(entry.get('level_name', '')).strip()
        classification = self._classify_entity(name, entry.get('classification'))

        return {
            'name': name,
            'x': x,
            'y': y,
            'level_name': level_name,
            'classification': classification,
            'active': bool(entry.get('active', True)),
        }

    def _classify_entity(self, name: str, classification: object):
        classification_text = str(classification).strip().lower() if classification else ''
        if classification_text:
            return classification_text

        lowered = name.lower()
        if lowered.startswith('water_puddle'):
            return 'water_puddle'
        if lowered.startswith('intruder') or lowered.startswith('actor_human'):
            return 'intruder'
        if lowered.startswith('barrel'):
            return 'barrel'
        return 'obstacle'

    def _evaluate_robot(self, robot_name: str):
        robot = self._robots.get(robot_name)
        if robot is None:
            return

        nearest = self._find_nearest_entity(robot)
        if nearest is None:
            return

        entity, distance = nearest
        if distance > self.args.alert_distance_threshold:
            return

        is_blocking = int(robot['mode']) in BLOCKING_MODES
        if not is_blocking and not self.args.trigger_on_any_nearby:
            return

        self._publish_alert(robot_name, robot, entity, distance)

    def _find_nearest_entity(self, robot: Dict[str, object]):
        robot_level = str(robot.get('level_name', '')).strip()
        candidates = []
        for entity in self._entities.values():
            if not entity.get('active', True):
                continue

            entity_level = str(entity.get('level_name', '')).strip()
            if robot_level and entity_level and robot_level != entity_level:
                continue

            distance = self._distance(
                float(robot['x']),
                float(robot['y']),
                float(entity['x']),
                float(entity['y']))
            candidates.append((entity, distance))

        if not candidates:
            return None

        return min(candidates, key=lambda item: item[1])

    def _publish_alert(self, robot_name: str, robot: Dict[str, object],
                       entity: Dict[str, object], distance: float):
        alert_type = str(entity.get('classification', 'obstacle'))
        alert_key = (robot_name, entity['name'], alert_type)
        now = time.time()
        last_alert = self._last_alert_time.get(alert_key)
        if last_alert is not None and now - last_alert < self.args.alert_cooldown_seconds:
            return

        self._last_alert_time[alert_key] = now

        payload = {
            'timestamp': now,
            'robot_name': robot_name,
            'level_name': robot.get('level_name', ''),
            'obstacle_type': alert_type,
            'obstacle_name': entity['name'],
            'distance_estimate': round(distance, 3),
            'obstacle_position': {
                'x': round(float(entity['x']), 3),
                'y': round(float(entity['y']), 3),
            },
            'robot_position': {
                'x': round(float(robot['x']), 3),
                'y': round(float(robot['y']), 3),
            },
            'recommended_action': self._recommended_action(alert_type),
            'robot_mode': robot.get('mode_name', 'unknown'),
            'source': 'perception_bridge',
        }

        self.get_logger().warn(
            f"Alerting on {alert_type} '{entity['name']}' for {robot_name} at {distance:.2f} m")
        self._alert_pub.publish(String(data=json.dumps(payload)))

    def _recommended_action(self, alert_type: str):
        if alert_type == 'water_puddle':
            return 'Work Order: Mop required'
        if alert_type == 'intruder':
            return 'Security Alert: investigate immediately'
        if alert_type == 'barrel':
            return 'Dynamic blockage: replan or clear the path'
        return 'Investigate obstacle and replan'

    def _distance(self, x0: float, y0: float, x1: float, y1: float):
        return math.hypot(x1 - x0, y1 - y0)

    def _mode_name(self, mode: int):
        names = {
            RobotMode.MODE_IDLE: 'idle',
            RobotMode.MODE_CHARGING: 'charging',
            RobotMode.MODE_MOVING: 'moving',
            RobotMode.MODE_PAUSED: 'paused',
            RobotMode.MODE_WAITING: 'waiting',
            RobotMode.MODE_EMERGENCY: 'emergency',
            RobotMode.MODE_GOING_HOME: 'going_home',
            RobotMode.MODE_DOCKING: 'docking',
            RobotMode.MODE_ADAPTER_ERROR: 'adapter_error',
        }
        return names.get(mode, f'unknown_{mode}')


def main(argv=sys.argv):
    rclpy.init(args=argv)
    node = PerceptionBridge(argv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
