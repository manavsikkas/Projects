from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'models'), glob('models/*.sdf')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='techie_kaneki',
    maintainer_email='techie_kaneki@todo.todo',
    description='Turtlebot hand gesture control with IMU and GNSS',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_gesture_detector.py = turtlebot.hand_gesture_detector:main',
            'robot_controller.py = turtlebot.robot_controller:main',
            'imu_simulator.py = turtlebot.imu_simulator:main',
            'gnss_mock.py = turtlebot.gnss_mock:main',
            'rmse_calculator.py = turtlebot.rmse_calculator:main',
        ],
    },
)
