from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robot_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name,
            ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='dell',
    maintainer_email='your@email.com',
    description='Overhead Localization',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = robot_hardware.vision.vision:main',
            'controller_node = robot_hardware.control.control:main',
            'communication_node = robot_hardware.control.communication:main',
            'path_planner_node = robot_hardware.control.path_planner:main',
            'vision_overlay = robot_hardware.vision.vision_overlay:main',
        ],
    },
)