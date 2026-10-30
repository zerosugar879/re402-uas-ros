from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robot_simulation'

def files(pattern):
    return [f for f in glob(pattern) if os.path.isfile(f)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), files('launch/*')),
        (os.path.join('share', package_name, 'config'), files('config/*')),
        (os.path.join('share', package_name, 'worlds'), files('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dell',
    maintainer_email='dell@todo.todo',
    description='Robot simulation launch and config',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_state_publisher = robot_simulation.gazebo_bridge.robot_state:main',
            'ball_state_publisher = robot_simulation.gazebo_bridge.ball_state:main',
            'obstacle_state_publisher = robot_simulation.gazebo_bridge.obstacle_state:main',
            'cmd_vel_state = robot_simulation.gazebo_bridge.cmd_vel_state:main',
        ],
    },
)
