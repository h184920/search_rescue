from setuptools import setup
import os 
from glob import glob

package_name = 'bug2_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rocotics',
    maintainer_email='rocotics@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wallfollower_controller = bug2_navigation.wallfollower3_controller:main',
            'go_to_point = bug2_navigation.go3point_controller:main',
            'bug2_controller = bug2_navigation.bug3_controller:main',
            'test_action_server = bug2_navigation.test_action_server:main',
            'dead = bug2_navigation.deadlockServer:main',
            'lock = bug2_navigation.deadlockClient:main',
        ],
    },
)
