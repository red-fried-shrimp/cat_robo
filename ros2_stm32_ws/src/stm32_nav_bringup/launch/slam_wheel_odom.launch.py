import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """启动使用 STM32 轮式里程计的建图模式。

    该入口会启动 STM32 通信节点，由 STM32 轮速积分发布 `/odom` 和
    `odom -> base_link`，同时关闭 `laser_odometry.py`，避免两个节点争抢同一条
    里程计/TF。
    """
    pkg_share = get_package_share_directory('stm32_nav_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'stm32_port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            description='STM32 serial port for wheel odometry'
        ),
        DeclareLaunchArgument(
            'lidar_port',
            default_value=(
                '/dev/serial/by-id/'
                'usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
                'f69570a8c213ec118f16f4ef7a109228-if00-port0'
            ),
            description='LiDAR serial port'
        ),
        DeclareLaunchArgument(
            'enable_stm32',
            default_value='true',
            description='Wheel odometry SLAM: start STM32 base communication node'
        ),
        DeclareLaunchArgument(
            'use_laser_odom',
            default_value='false',
            description='Wheel odometry SLAM: keep LiDAR odometry node disabled'
        ),

        LogInfo(msg=(
            '[SLAM] Wheel odometry mode: starting STM32 node, '
            'publishing /odom and odom->base_link from wheel feedback.'
        )),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'slam_launch.py')
            ),
            launch_arguments={
                'stm32_port': LaunchConfiguration('stm32_port'),
                'lidar_port': LaunchConfiguration('lidar_port'),
                'enable_stm32': LaunchConfiguration('enable_stm32'),
                'use_laser_odom': LaunchConfiguration('use_laser_odom'),
            }.items()
        ),
    ])
