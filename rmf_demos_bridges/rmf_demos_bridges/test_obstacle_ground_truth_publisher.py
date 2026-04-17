import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TestObstacleGroundTruthPublisher(Node):
    def __init__(self, argv=sys.argv):
        parser = argparse.ArgumentParser()
        parser.add_argument('--topic', default='/obstacle_ground_truth',
                            help='Ground-truth topic consumed by test_obstacle_classifier')
        parser.add_argument('--robot-name', required=True,
                            help='Robot name to associate with the ground-truth label')
        parser.add_argument('--obstacle-type', required=True, choices=['puddle', 'intruder'],
                            help='Ground-truth obstacle type to publish')
        parser.add_argument('--object-id', default='',
                            help='Optional stable id for the test object')
        parser.add_argument('--confidence', type=float, default=1.0,
                            help='Confidence value for the manual label')
        parser.add_argument('--repeat-rate-hz', type=float, default=0.0,
                            help='If > 0, keep republishing at this rate until stopped')

        self.args, _ = parser.parse_known_args(argv[1:])
        super().__init__('test_obstacle_ground_truth_publisher')

        self._publisher = self.create_publisher(String, self.args.topic, 10)
        self._payload = {
            'timestamp': time.time(),
            'source': 'test_obstacle_ground_truth_publisher',
            'robot_name': self.args.robot_name,
            'obstacle_type': self.args.obstacle_type,
            'object_id': self.args.object_id,
            'confidence': self.args.confidence,
        }

        self._publish_once()
        if self.args.repeat_rate_hz > 0.0:
            period = 1.0 / self.args.repeat_rate_hz
            self._timer = self.create_timer(period, self._publish_once)
        else:
            self._timer = None

        self.get_logger().info(
            f"Publishing ground truth label on {self.args.topic}: "
            f"robot={self.args.robot_name}, type={self.args.obstacle_type}, object_id={self.args.object_id}")

    def _publish_once(self):
        self._payload['timestamp'] = time.time()
        self._publisher.publish(String(data=json.dumps(self._payload)))


def main(argv=sys.argv):
    rclpy.init(args=argv)
    node = TestObstacleGroundTruthPublisher(argv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
