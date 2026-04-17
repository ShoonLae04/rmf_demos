#!/usr/bin/env python3

from pathlib import Path
import sys


PLUGIN_SNIPPET = """    <plugin filename=\"libignition-gazebo-sensors-system.so\" name=\"ignition::gazebo::systems::Sensors\">
    </plugin>
"""


def insert_plugin(world_path: Path):
    text = world_path.read_text()
    if 'ignition::gazebo::systems::Sensors' in text:
        return

    marker = '    <plugin filename="libignition-gazebo-scene-broadcaster-system.so" name="ignition::gazebo::systems::SceneBroadcaster">\n    </plugin>\n'
    if marker not in text:
        raise RuntimeError(f'Could not find SceneBroadcaster plugin block in {world_path}')

    text = text.replace(marker, marker + PLUGIN_SNIPPET, 1)
    world_path.write_text(text)


def main(argv=None):
    argv = argv or sys.argv
    if len(argv) != 2:
        raise SystemExit('Usage: insert_ign_sensors_plugin.py <world_path>')

    world_path = Path(argv[1])
    if not world_path.exists():
        raise SystemExit(f'World file does not exist: {world_path}')

    insert_plugin(world_path)


if __name__ == '__main__':
    main()
