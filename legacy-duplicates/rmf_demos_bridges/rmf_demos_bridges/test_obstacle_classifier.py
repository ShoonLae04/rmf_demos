import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rmf_task_msgs.msg import DispatchStates, ApiResponse


class TestObstacleClassifier(Node):
    def __init__(self, argv=sys.argv):
        parser = argparse.ArgumentParser()
        parser.add_argument('--input-alert-topic', default='/rmf_demo_alerts',
                            help='Input alert topic from scan_obstacle_detector')
        parser.add_argument('--output-classification-topic', default='/obstacle_classifications',
                            help='Output classification topic consumed by scan_obstacle_detector')
        parser.add_argument('--ground-truth-topic', default='/obstacle_ground_truth',
                            help='Topic carrying explicit ground-truth obstacle labels')
        parser.add_argument('--dispatch-states-topic', default='/dispatch_states',
                    help='DispatchStates topic used to infer active clean task assignments')
        parser.add_argument('--task-api-response-topic', default='/task_api_responses',
                    help='ApiResponse topic used to discover clean task IDs')
        parser.add_argument('--mode', default='perception_first', choices=['perception_first', 'task_aware', 'manual', 'fixed', 'distance', 'robot'],
                            help='Classification mode for test labels')
        parser.add_argument('--task-aware-clean-type', default='puddle',
                    help='Type emitted in task_aware mode when robot has an active clean task')
        parser.add_argument('--task-aware-idle-type', default='unknown_obstacle',
                    help='Type emitted in task_aware mode when robot has no active clean task')
        parser.add_argument('--fixed-type', default='intruder', choices=['intruder', 'puddle'],
                            help='Type used when mode=fixed')
        parser.add_argument('--distance-intruder-threshold', type=float, default=0.8,
                            help='When mode=distance, distance <= threshold is intruder, else puddle')
        parser.add_argument('--label-map-json', default='',
                            help='JSON map of robot_name or object_id to obstacle_type for manual testing')
        parser.add_argument('--robot-name', default='',
                            help='Optional robot name filter. If set, only alerts from this robot are classified')
        parser.add_argument('--cooldown-sec', type=float, default=1.0,
                            help='Minimum interval between classifications per robot')
        parser.add_argument('--confidence', type=float, default=0.9,
                            help='Confidence value to include in output labels')
        parser.add_argument('--relabel-known', action='store_true', default=False,
                            help='If set, also relabel alerts that already have a non-unknown obstacle_type')

        self.args, _ = parser.parse_known_args(argv[1:])
        super().__init__('test_obstacle_classifier')

        self._publisher = self.create_publisher(String, self.args.output_classification_topic, 10)
        self._subscriber = self.create_subscription(
            String,
            self.args.input_alert_topic,
            self._alert_callback,
            10)
        self._dispatch_states_subscriber = self.create_subscription(
            DispatchStates,
            self.args.dispatch_states_topic,
            self._dispatch_states_callback,
            10)
        self._task_api_response_subscriber = self.create_subscription(
            ApiResponse,
            self.args.task_api_response_topic,
            self._task_api_response_callback,
            10)
        self._ground_truth_subscriber = self.create_subscription(
            String,
            self.args.ground_truth_topic,
            self._ground_truth_callback,
            10)

        self._last_pub_time = {}
        self._label_map = {}
        self._ground_truth_labels = {}
        self._active_clean_tasks = {}
        self._clean_task_ids = set()

        if self.args.label_map_json:
            try:
                self._label_map = json.loads(self.args.label_map_json)
            except Exception as err:
                self.get_logger().warn(
                    f'Failed to parse --label-map-json: {err}. Continuing without label map.')

        self.get_logger().info(
            f'Test obstacle classifier active: mode={self.args.mode}, '
            f'input={self.args.input_alert_topic}, output={self.args.output_classification_topic}, '
            f'ground_truth={self.args.ground_truth_topic}, '
            f'dispatch_states={self.args.dispatch_states_topic}, '
            f'task_api_responses={self.args.task_api_response_topic}')

    def _robot_key(self, robot_name: str) -> str:
        if not robot_name:
            return ''
        return robot_name.strip().lower()

    def _classify(self, robot_name: str, distance: float, incoming_type: str) -> tuple[str, str]:
        normalized_type = (incoming_type or '').strip().lower()

        if self.args.mode == 'perception_first':
            if normalized_type and normalized_type != 'unknown_obstacle':
                return normalized_type, 'perception_first_passthrough'
            return self.args.task_aware_idle_type, 'perception_first_unknown'

        if self.args.mode == 'task_aware':
            robot_key = self._robot_key(robot_name)
            if robot_key in self._active_clean_tasks:
                return self.args.task_aware_clean_type, 'task_aware_active_clean_task'
            return self.args.task_aware_idle_type, 'task_aware_not_clean_task'

        if self.args.mode == 'manual':
            robot_key = self._robot_key(robot_name)
            if robot_key in self._ground_truth_labels:
                label = self._ground_truth_labels[robot_key]
                return label['obstacle_type'], 'manual_ground_truth_robot'

            if robot_name in self._label_map:
                return self._label_map[robot_name], 'manual_label_map_robot'

            if robot_key in self._label_map:
                return self._label_map[robot_key], 'manual_label_map_robot_norm'

            return 'unknown_obstacle', 'manual_no_label'

        if self.args.mode == 'fixed':
            return self.args.fixed_type, 'fixed_mode'

        if self.args.mode == 'distance':
            if distance <= self.args.distance_intruder_threshold:
                return 'intruder', 'distance_rule_near'
            return 'puddle', 'distance_rule_far'

        # mode == robot
        if 'cleaner' in self._robot_key(robot_name):
            return 'puddle', 'robot_rule_cleaner'
        return 'intruder', 'robot_rule_other'

    def _task_api_response_callback(self, msg: ApiResponse):
        try:
            payload = json.loads(msg.json_msg)
        except Exception:
            return

        state = payload.get('state', {})
        if not isinstance(state, dict):
            return

        category = state.get('category', '')
        booking = state.get('booking', {})
        if not isinstance(booking, dict):
            return

        task_id = booking.get('id', '')
        if not task_id:
            return

        if category == 'clean':
            self._clean_task_ids.add(task_id)

    def _dispatch_states_callback(self, msg: DispatchStates):
        active_clean_robots = {}

        for state in msg.active:
            if not state.assignment.is_assigned:
                continue

            robot_name = state.assignment.expected_robot_name.strip()
            if not robot_name:
                continue

            # STATUS_SELECTED=2 and STATUS_DISPATCHED=3
            if state.status not in (2, 3):
                continue

            is_clean_task = (
                state.task_id in self._clean_task_ids or
                state.task_id.startswith('clean.') or
                state.task_id.startswith('clean_')
            )
            if not is_clean_task:
                continue

            robot_key = self._robot_key(robot_name)
            active_clean_robots[robot_key] = {
                'task_id': state.task_id,
                'fleet_name': state.assignment.fleet_name,
                'timestamp': time.time(),
            }

        self._active_clean_tasks = active_clean_robots

    def _ground_truth_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return

        robot_name = payload.get('robot_name', '')
        obstacle_type = payload.get('obstacle_type', '')
        if not robot_name or not obstacle_type:
            return

        confidence = float(payload.get('confidence', self.args.confidence))
        robot_key = self._robot_key(robot_name)
        self._ground_truth_labels[robot_key] = {
            'obstacle_type': obstacle_type,
            'confidence': confidence,
            'object_id': payload.get('object_id', ''),
            'timestamp': time.time(),
        }

        self.get_logger().info(
            f'Ground truth label updated for {robot_name}: {obstacle_type} '
            f'(object_id={payload.get("object_id", "")})')

    def _alert_callback(self, msg: String):
        try:
            alert = json.loads(msg.data)
        except Exception:
            return

        robot_name = alert.get('robot_name', '')
        if not robot_name:
            return

        if self.args.robot_name and robot_name != self.args.robot_name:
            return

        obstacle_type = alert.get('obstacle_type', 'unknown_obstacle')
        if not self.args.relabel_known and obstacle_type != 'unknown_obstacle' \
            and self.args.mode not in ('manual', 'perception_first'):
            return

        distance = float(alert.get('distance_estimate', 999.0))
        label, reason = self._classify(robot_name, distance, obstacle_type)

        now = time.time()
        key = self._robot_key(robot_name)
        last = self._last_pub_time.get(key, 0.0)
        if now - last < max(0.0, self.args.cooldown_sec):
            return
        self._last_pub_time[key] = now

        payload = {
            'timestamp': now,
            'source': 'test_obstacle_classifier',
            'robot_name': robot_name,
            'obstacle_type': label,
            'confidence': self.args.confidence,
            'reason': reason,
            'distance_estimate': distance,
            'sector': alert.get('sector', ''),
            'scan_topic': alert.get('scan_topic', ''),
            'frame_id': alert.get('frame_id', ''),
        }

        if label == 'unknown_obstacle' and self.args.mode == 'manual':
            payload['note'] = 'No manual ground truth was provided for this robot yet.'

        self._publisher.publish(String(data=json.dumps(payload)))
        self.get_logger().info(
            f"Published test label: robot={robot_name}, type={label}, reason={reason}, "
            f"distance={distance:.2f}")


def main(argv=sys.argv):
    rclpy.init(args=argv)
    node = TestObstacleClassifier(argv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
