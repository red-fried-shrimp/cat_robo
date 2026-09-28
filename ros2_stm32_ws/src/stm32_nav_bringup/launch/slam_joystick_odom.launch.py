import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """启动手柄控制建图模式。

    STM32 继续由手柄/下位机控制；上位机只读取 STM32 上报的轮速里程计，
    发布 `/odom` 和 `odom -> base_link`，不发送模式切换或速度控制帧。
    """
    pkg_share = get_package_share_directory('stm32_nav_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'stm32_port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            description='STM32 serial port for read-only wheel odometry'
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

        LogInfo(msg=(
            '[SLAM] Joystick odometry mode: STM32 serial is read-only; '
            'no mode or velocity frames will be sent from ROS.'
        )),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'slam_launch.py')
            ),
            launch_arguments={
                'stm32_port': LaunchConfiguration('stm32_port'),
                'lidar_port': LaunchConfiguration('lidar_port'),
                'enable_stm32': 'true',
                'stm32_read_only': 'true',
                'use_laser_odom': 'false',
            }.items()
        ),
    ])
