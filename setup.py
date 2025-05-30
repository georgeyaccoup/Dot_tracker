from setuptools import setup, find_packages

package_name = 'small_project'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Color tracking and motor control package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_detector = small_project.color_detector:main',
            'kinematics = small_project.kinematics:main',
            'pid_controller = small_project.pid_controller:main',
            'motor_x = small_project.Motor_x:main',
            'motor_y = small_project.Motor_y:main',
        ],
    },
)
