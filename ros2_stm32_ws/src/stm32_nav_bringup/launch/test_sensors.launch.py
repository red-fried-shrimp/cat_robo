import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """启动传感器和 RViz，用于不跑 SLAM/Nav2 时检查硬件数据。

    RViz 配置的 Fixed Frame 是 `map`，但传感器单测不会启动 slam_toolbox 或 AMCL。
    因此这里默认不启动 STM32 底盘通信，只打开临时 `map -> odom` 和
    `odom -> base_link` 静态 TF，避免单测传感器时误触底盘运动。
    """
    pkg_share = get_package_share_directory('stm32_nav_bringup')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    return LaunchDescription([
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
        DeclareLaunchArgument('use_laser_odom', default_value='false'),
        DeclareLaunchArgument(
            'enable_stm32',
            default_value='false',
            description='Sensor test safety default: do not start STM32 base node'
        ),
        DeclareLaunchArgument(
            'publish_identity_map_odom',
            default_value='true',
            description='Sensor test only: publish temporary static map->odom TF'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'sensors.launch.py')
            ),
            launch_arguments={
                'stm32_port': LaunchConfiguration('stm32_port'),
                'lidar_port': LaunchConfiguration('lidar_port'),
                'use_laser_odom': LaunchConfiguration('use_laser_odom'),
                'enable_stm32': LaunchConfiguration('enable_stm32'),
                'publish_identity_map_odom': LaunchConfiguration('publish_identity_map_odom'),
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
