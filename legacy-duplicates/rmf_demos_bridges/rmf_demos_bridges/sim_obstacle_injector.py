import argparse
import json
import math
import shutil
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String


PRESET_MODELS = {
    'water_puddle': {
        'classification': 'water_puddle',
        'model_relpath': Path('models/water_puddle/model.sdf'),
    },
    'puddle': {
        'classification': 'puddle',
        'model_relpath': Path('models/water_puddle/model.sdf'),
    },
    'spill': {
        'classification': 'spill',
        'model_relpath': Path('models/water_puddle/model.sdf'),
    },
    'stain': {
        'classification': 'stain',
        'model_relpath': Path('models/water_puddle/model.sdf'),
    },
    'litter': {
        'classification': 'litter',
        'model_relpath': Path('models/barrel/model.sdf'),
    },
    'leaf': {
        'classification': 'leaf',
        'model_relpath': Path('models/barrel/model.sdf'),
    },
    'hair': {
        'classification': 'hair',
        'model_relpath': Path('models/barrel/model.sdf'),
    },
    'barrel': {
        'classification': 'barrel',
        'model_relpath': Path('models/barrel/model.sdf'),
    },
}


class SimObstacleInjector(Node):
    def __init__(self, argv=sys.argv):
        parser = argparse.ArgumentParser()
        parser.add_argument('--kind', choices=sorted(PRESET_MODELS.keys()), default='water_puddle')
        parser.add_argument('--name', default=None)
        parser.add_argument('--x', type=float, default=0.0)
        parser.add_argument('--y', type=float, default=0.0)
        parser.add_argument('--z', type=float, default=0.0)
        parser.add_argument('--yaw', type=float, default=0.0)
        parser.add_argument('--level-name', default='L1')
        parser.add_argument('--register-topic', default='/sim_injected_entities')
        parser.add_argument('--spawn', action='store_true', help='Try to spawn the model through Ignition service.')
        parser.add_argument('--service', default=None, help='Override Ignition create service path.')
        parser.add_argument('--world', default=None, help='World name used to build the default Ignition service path.')
        parser.add_argument('--model-path', default=None, help='Override the model.sdf path to spawn.')
        parser.add_argument('--repeat', type=int, default=3, help='Number of metadata publishes to send.')
        parser.add_argument('--rate', type=float, default=4.0, help='Metadata publish rate in Hz while repeating.')

        self.args = parser.parse_args(argv[1:])
        super().__init__('sim_obstacle_injector')

        self._publisher = self.create_publisher(String, self.args.register_topic, 10)

    def run(self):
        model_path = self._resolve_model_path()
        entity_name = self.args.name or f"{self.args.kind}_{int(time.time())}"
        payload = self._build_payload(entity_name, model_path)

        if self.args.spawn:
            self._try_spawn(entity_name, model_path)

        self._publish_payload(payload)
        self.get_logger().info(
            f"Registered {self.args.kind} as {entity_name} on {self.args.register_topic}")

    def _resolve_model_path(self):
        if self.args.model_path:
            return Path(self.args.model_path).expanduser().resolve()

        assets_share = Path(get_package_share_directory('rmf_demos_assets'))
        model_path = assets_share / PRESET_MODELS[self.args.kind]['model_relpath']
        if not model_path.exists():
            raise FileNotFoundError(f'Model file not found: {model_path}')
        return model_path

    def _build_payload(self, entity_name: str, model_path: Path):
        classification = PRESET_MODELS[self.args.kind]['classification']
        payload = {
            'name': entity_name,
            'classification': classification,
            'level_name': self.args.level_name,
            'x': self.args.x,
            'y': self.args.y,
            'z': self.args.z,
            'yaw': self.args.yaw,
            'model_path': str(model_path),
            'active': True,
        }
        return payload

    def _publish_payload(self, payload):
        message = String()
        message.data = json.dumps({'entities': [payload]})
        delay = 1.0 / max(self.args.rate, 0.1)
        for _ in range(max(self.args.repeat, 1)):
            self._publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(delay)

    def _try_spawn(self, entity_name: str, model_path: Path):
        service = self.args.service
        if not service:
            world = self.args.world or 'sim_world'
            service = f'/world/{world}/create'

        quaternion_z = math.sin(self.args.yaw / 2.0)
        quaternion_w = math.cos(self.args.yaw / 2.0)
        request = (
            'sdf_filename: "{model_path}" '
            'name: "{entity_name}" '
            'allow_renaming: false '
            'pose: {{position: {{x: {x} y: {y} z: {z}}} '
            'orientation: {{x: 0 y: 0 z: {qz} w: {qw}}}}}'
        ).format(
            model_path=model_path,
            entity_name=entity_name,
            x=self.args.x,
            y=self.args.y,
            z=self.args.z,
            qz=quaternion_z,
            qw=quaternion_w,
        )

        ign = shutil.which('ign')
        if ign is None:
            self.get_logger().warn('ign command not found; falling back to registration only')
            return

        command = [
            ign, 'service',
            '-s', service,
            '--reqtype', 'ignition.msgs.EntityFactory',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '3000',
            '--req', request,
        ]
        self.get_logger().info('Attempting Ignition spawn via: ' + ' '.join(command))
        try:
            completed = subprocess.run(command, check=True, capture_output=True, text=True)
            if completed.stdout.strip():
                self.get_logger().info(completed.stdout.strip())
        except subprocess.CalledProcessError as exc:
            self.get_logger().warn(
                f'Ignition spawn failed, continuing with registration only: {exc.stderr.strip()}')


def main(argv=sys.argv):
    rclpy.init(args=argv)
    node = SimObstacleInjector(argv)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)