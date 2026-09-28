import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    LogInfo, OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def validate_map(context):
    """提示导航模式必须传入地图文件。

    AMCL 只有拿到地图并拥有初始位姿后，才会发布真正的 `map -> odom`。
    """
    map_path = context.launch_configurations.get('map', '')
    if not map_path:
        print()
        print('WARNING: No map file provided! Navigation requires an existing map.')
        print('  Usage: ros2 launch stm32_nav_bringup nav_launch.py map:=<path_to_map.yaml>')
        print()
    return []


def generate_launch_description():
    """启动传感器、Nav2、AMCL 和 RViz。

    `map -> odom` 在导航模式中由 AMCL 动态发布。这里不启用传感器单测用的
    临时静态 `map -> odom`，避免和 AMCL 抢同一条 TF。
    """
    pkg_share = get_package_share_directory('stm32_nav_bringup')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='',
                              description='Path to map YAML file (required)'),
        DeclareLaunchArgument(
            'stm32_port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
        ),
        DeclareLaunchArgument(
            'lidar_port',
            default_value=(
                '/dev/serial/by-id/'
                'usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
                'f69570a8c213ec118f16f4ef7a109228-if00-port0'
            )
        ),
        DeclareLaunchArgument(
            'use_laser_odom', default_value='false',
            description='Use LiDAR-based odometry instead of STM32 wheel odometry'
        ),
        DeclareLaunchArgument(
            'enable_stm32', default_value='true',
            description='Start STM32 base communication node'
        ),

        OpaqueFunction(function=validate_map),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'sensors.launch.py')
            ),
            launch_arguments={
                'stm32_port': LaunchConfiguration('stm32_port'),
                'lidar_port': LaunchConfiguration('lidar_port'),
                'use_laser_odom': LaunchConfiguration('use_laser_odom'),
                'enable_stm32': LaunchConfiguration('enable_stm32'),
                'publish_identity_map_odom': 'false',
            }.items()
        ),

        # LiDAR 里程计（仅 use_laser_odom=true 时启动）
        Node(
            package='stm32_nav_bringup',
            executable='laser_odometry.py',
            name='laser_odometry',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_laser_odom'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch',
                    'bringup_launch.py'
                )
            ),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': 'false',
                'params_file': nav2_params,
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])
