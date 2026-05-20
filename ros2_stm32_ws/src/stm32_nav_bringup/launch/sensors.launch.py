import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取 stm32_nav_bringup 功能包路径
    pkg_share = get_package_share_directory('stm32_nav_bringup')

    # 定义可在启动时传入的串口参数
    stm32_port = LaunchConfiguration('stm32_port')
    lidar_port = LaunchConfiguration('lidar_port')

    # 各个传感器的参数文件路径
    stm32_param_file = os.path.join(pkg_share, 'config', 'stm32_comm.yaml')
    lidar_param_file = os.path.join(pkg_share, 'config', 'sllidar_s2.yaml')
    imu_param_file = os.path.join(pkg_share, 'config', 'imu_h30.yaml')

    return LaunchDescription([
        # 声明可配置的 STM32 串口参数
        DeclareLaunchArgument(
            'stm32_port',
            default_value='/dev/ttyUSB0',
            description='STM32 serial port'
        ),
        # 声明可配置的雷达串口参数
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/ttyUSB1',
            description='LiDAR serial port'
        ),

        # STM32 底盘通信节点：发布里程计、速度控制、TF
        Node(
            package='stm32_nav_comm',
            executable='stm32_comm_node',
            name='stm32_comm_node',
            output='screen',
            parameters=[
                stm32_param_file,
                {
                    'stm32_serial_port': stm32_port,
                    'publish_tf': True,
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                }
            ]
        ),

        # IMU 传感器节点
        Node(
            package='yesense_std_ros2',
            executable='yesense_node_publisher',
            name='yesense_pub',
            output='screen',
            parameters=[imu_param_file]
        ),

        # 激光雷达节点
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[
                lidar_param_file,
                {
                    'serial_port': lidar_port
                }
            ]
        ),

        # 静态坐标变换：base_link -> gyro_link（IMU 坐标系）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_tf',
            output='screen',
            arguments=[
                '0', '0', '-0.01805',
                '0', '0', '0',
                'base_link',
                'gyro_link'
            ]
        ),

        # 静态坐标变换：base_link -> laser（雷达坐标系）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            output='screen',
            arguments=[
                '0', '0', '0.54409',
                '0', '0', '0',
                'base_link',
                'laser'
            ]
        ),
    ])

