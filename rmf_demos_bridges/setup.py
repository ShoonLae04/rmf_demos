from setuptools import setup

package_name = 'rmf_demos_bridges'

setup(
    name=package_name,
    version='2.0.3',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Boon Han',
    maintainer_email='cnboonhan@openrobotics.org',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_socketio_bridge=rmf_demos_bridges.fleet_socketio_bridge:main',
            'sim_obstacle_injector=rmf_demos_bridges.sim_obstacle_injector:main',
            'perception_bridge=rmf_demos_bridges.perception_bridge:main',
            'scan_obstacle_detector=rmf_demos_bridges.scan_obstacle_detector:main',
            'test_obstacle_classifier=rmf_demos_bridges.test_obstacle_classifier:main',
            'test_obstacle_ground_truth_publisher=rmf_demos_bridges.test_obstacle_ground_truth_publisher:main',
            'incident_task_dispatcher=rmf_demos_bridges.incident_task_dispatcher:main',
            'fleet_robotmanager_mqtt_bridge=rmf_demos_bridges.fleet_robotmanager_mqtt_bridge:main',
        ],
    },
)
