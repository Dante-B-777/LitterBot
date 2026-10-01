import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'litterbot_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'litterbot_bridge': ['ui/index.html'],
    },
    data_files=[
        # Required ament resource index marker
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Static config/reference files
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools', 'numpy', 'matplotlib'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Benji',
    maintainer_email='benji@litterbot.local',
    description=(
        'Bridge node between ROS2 and the LitterBot STM32 MCU over UART. '
        'Translates /cmd_vel Twist commands into mecanum wheel motor percentages '
        'and publishes raw encoder ticks from the STM32.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            # stm32_bridge: main bridge node
            #   ros2 run litterbot_bridge stm32_bridge
            #   ros2 run litterbot_bridge stm32_bridge --ros-args -p serial_port:=/dev/ttyUSB0
            'stm32_bridge = litterbot_bridge.stm32_bridge:main',
            # odom_publisher: publish /odom from encoder ticks or cmd_vel fallback
            'odom_publisher = litterbot_bridge.odom_publisher:main',
            # imu_bridge: normalize RealSense IMU topics to a stable /imu/data_raw stream
            'imu_bridge = litterbot_bridge.imu_bridge:main',
            # ekf_localization: modular inspectable localization filter
            'ekf_localization = litterbot_bridge.ekf_localization_node:main',
            # localization_calibrator: stationary noise and bias estimator
            'localization_calibrator = litterbot_bridge.localization_calibrator:main',
            # localization_test: drive a commanded trajectory and compare /odometry/ekf to ideal motion
            'localization_test = litterbot_bridge.localization_test_runner:main',
            # ui_server: command center WebSocket bridge
            #   ros2 run litterbot_bridge ui_server
            #   ros2 run litterbot_bridge ui_server --ros-args -p port:=8765
            'ui_server = litterbot_bridge.ui_server:main',
        ],
    },
)
