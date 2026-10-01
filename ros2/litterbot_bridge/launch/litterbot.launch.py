"""
litterbot.launch.py - Start all LitterBot ROS2 nodes with one command.

Usage:
    ros2 launch litterbot_bridge litterbot.launch.py

    # Real hardware with auto-detected serial devices:
    ros2 launch litterbot_bridge litterbot.launch.py mock_serial:=false

    # Explicit overrides still work:
    ros2 launch litterbot_bridge litterbot.launch.py \
        mock_serial:=false serial_port:=/dev/ttySTM32 lidar_port:=/dev/ttyLIDAR
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from litterbot_bridge.device_discovery import detect_serial_device


def _is_enabled(value: str) -> bool:
    return value.strip().lower() not in {'0', 'false', 'no', 'off'}


def _launch_setup(context, *args, **kwargs):
    serial_override = LaunchConfiguration('serial_port').perform(context)
    lidar_override = LaunchConfiguration('lidar_port').perform(context)
    mock_serial = LaunchConfiguration('mock_serial').perform(context)
    enable_lidar = LaunchConfiguration('enable_lidar').perform(context)
    enable_realsense_imu = LaunchConfiguration('enable_realsense_imu').perform(context)
    enable_localization = LaunchConfiguration('enable_localization').perform(context)

    stm32_resolution = detect_serial_device('stm32', preferred=serial_override)
    lidar_resolution = detect_serial_device(
        'lidar',
        preferred=lidar_override,
        exclude_paths=[stm32_resolution.path],
    )

    resolved_stm32_port = stm32_resolution.path or serial_override or '/dev/ttyACM0'
    actions = [
        LogInfo(
            msg=(
                f'LitterBot launch: STM32 port={resolved_stm32_port} '
                f'({stm32_resolution.source}, mock={mock_serial})'
            )
        ),
        Node(
            package='litterbot_bridge',
            executable='stm32_bridge',
            name='stm32_bridge',
            parameters=[{
                'serial_port': resolved_stm32_port,
                'mock_serial': LaunchConfiguration('mock_serial'),
            }],
            output='screen',
        ),
        Node(
            package='litterbot_bridge',
            executable='ui_server',
            name='ui_server',
            parameters=[{
                'port': LaunchConfiguration('ui_port'),
            }],
            output='screen',
        ),
        Node(
            package='litterbot_bridge',
            executable='odom_publisher',
            name='odom_publisher',
            output='screen',
        ),
        Node(
            package='litterbot_vision',
            executable='camera_node',
            name='camera_node',
            condition=IfCondition(LaunchConfiguration('enable_camera')),
            parameters=[{
                'device': LaunchConfiguration('camera_device'),
                'width': LaunchConfiguration('camera_width'),
                'height': LaunchConfiguration('camera_height'),
                'fps': LaunchConfiguration('camera_fps'),
            }],
            output='screen',
        ),
        Node(
            package='litterbot_bridge',
            executable='imu_bridge',
            name='imu_bridge',
            condition=IfCondition(LaunchConfiguration('enable_realsense_imu')),
            output='screen',
        ),
        Node(
            package='litterbot_bridge',
            executable='ekf_localization',
            name='ekf_localization',
            condition=IfCondition(LaunchConfiguration('enable_localization')),
            output='screen',
        ),
    ]

    if _is_enabled(enable_realsense_imu):
        actions.append(LogInfo(msg='LitterBot launch: starting RealSense IMU driver'))
        actions.append(
            Node(
                package='realsense2_camera',
                executable='realsense2_camera_node',
                name='realsense_imu',
                parameters=[{
                    'enable_color': False,
                    'enable_depth': False,
                    'enable_infra1': False,
                    'enable_infra2': False,
                    'enable_gyro': True,
                    'enable_accel': True,
                    'unite_imu_method': 2,
                }],
                output='screen',
            )
        )

    if _is_enabled(enable_lidar):
        if lidar_resolution.path:
            actions.append(
                LogInfo(
                    msg=(
                        f'LitterBot launch: LiDAR port={lidar_resolution.path} '
                        f'({lidar_resolution.source})'
                    )
                )
            )
            actions.append(
                Node(
                    package='rplidar_ros',
                    executable='rplidar_node',
                    name='rplidar_node',
                    parameters=[{
                        'serial_port': lidar_resolution.path,
                        'serial_baudrate': LaunchConfiguration('lidar_baudrate'),
                        'frame_id': 'laser',
                        'inverted': False,
                        'angle_compensate': True,
                    }],
                    output='screen',
                )
            )
        else:
            actions.append(LogInfo(msg='LitterBot launch: LiDAR enabled but no matching serial device was found; skipping rplidar_node'))

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='auto'),
        DeclareLaunchArgument('mock_serial', default_value='true'),
        DeclareLaunchArgument('ui_port', default_value='8765'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('camera_device', default_value='4'),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_height', default_value='480'),
        DeclareLaunchArgument('camera_fps', default_value='10.0'),
        DeclareLaunchArgument('enable_realsense_imu', default_value='true'),
        DeclareLaunchArgument('enable_localization', default_value='false'),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('lidar_port', default_value='auto'),
        DeclareLaunchArgument('lidar_baudrate', default_value='115200'),
        OpaqueFunction(function=_launch_setup),
    ])
