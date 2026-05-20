import os  # 导入系统路径模块

# 导入ROS2启动相关模块
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取功能包共享目录路径（定位配置文件）
    pkg_share = get_package_share_directory('stm32_nav_bringup')
    # 拼接slam_toolbox参数文件路径
    slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    # 拼接RViz配置文件路径
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    # 构建启动描述
    return LaunchDescription([
        # 声明启动参数：STM32串口（默认/dev/ttyUSB0）
        DeclareLaunchArgument('stm32_port', default_value='/dev/ttyUSB0'),
        # 声明启动参数：激光雷达串口（默认/dev/ttyUSB1）
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1'),

        # 核心修改：设置环境变量，强制RViz使用软件渲染（解决GLSL报错）
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),

        # 包含传感器启动文件（STM32+IMU+激光雷达）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'sensors.launch.py')
            ),
            # 传递启动参数给传感器启动文件
            launch_arguments={
                'stm32_port': LaunchConfiguration('stm32_port'),
                'lidar_port': LaunchConfiguration('lidar_port'),
            }.items()
        ),

        # 启动slam_toolbox节点（异步建图）
        Node(
            package='slam_toolbox',          # 功能包名
            executable='async_slam_toolbox_node', # 可执行文件
            name='slam_toolbox',             # 节点名
            output='screen',                 # 输出到终端
            parameters=[slam_params]         # 加载参数文件
        ),

        # 启动RViz2节点（可视化）
        Node(
            package='rviz2',                 # 功能包名
            executable='rviz2',              # 可执行文件
            name='rviz2',                    # 节点名
            arguments=['-d', rviz_config],   # 加载RViz配置文件
            output='screen'                  # 输出到终端
        )
    ])

