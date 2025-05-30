from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'focus_v2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='george_pi',
    maintainer_email='george_pi@example.com',
    description='ROS2 package for controlling a 2-DOF robot arm with color detection',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_detector = focus_v2.color_detector:main',
            'kinematics = focus_v2.kinematics:main',
            'pid_controller = focus_v2.pid_controller:main',
            'motor_x = focus_v2.motor_x:main',
            'motor_y = focus_v2.motor_y:main',
        ],
    },
)