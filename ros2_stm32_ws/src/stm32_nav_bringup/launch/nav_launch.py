import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取 stm32_nav_bringup 功能包的共享路径
    pkg_share = get_package_share_directory('stm32_nav_bringup')
    
    # Nav2 参数配置文件路径
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    # RViz 可视化配置文件路径
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    return LaunchDescription([
        # 声明启动参数：地图文件路径
        DeclareLaunchArgument('map', default_value=''),
        
        # 声明启动参数：STM32 串口
        DeclareLaunchArgument('stm32_port', default_value='/dev/ttyUSB0'),
        
        # 声明启动参数：雷达串口
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1'),

        # 包含并启动传感器与底盘驱动启动文件（底盘、IMU、雷达、TF）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'sensors.launch.py')
            ),
            launch_arguments={
                'stm32_port': LaunchConfiguration('stm32_port'),
                'lidar_port': LaunchConfiguration('lidar_port'),
            }.items()
        ),

        # 包含并启动官方 Nav2 导航启动文件
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
                'params_file': nav2_params
            }.items()
        ),

        # 启动 RViz 可视化工具
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])

