import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """启动底盘、IMU、雷达和机器人本体静态 TF。

    这里只负责产生 `odom -> base_link` 以及 `base_link -> sensor` 这一段 TF。
    `map -> odom` 正常应由 slam_toolbox 或 AMCL 发布；传感器单测时可显式打开
    `publish_identity_map_odom`，临时补一个 map 原点到 odom 原点的静态 TF。
    """
    # 获取 stm32_nav_bringup 功能包路径
    pkg_share = get_package_share_directory('stm32_nav_bringup')

    # 定义可在启动时传入的串口参数
    stm32_port = LaunchConfiguration('stm32_port')
    lidar_port = LaunchConfiguration('lidar_port')
    use_laser_odom = LaunchConfiguration('use_laser_odom')
    enable_stm32 = LaunchConfiguration('enable_stm32')
    stm32_read_only = LaunchConfiguration('stm32_read_only')
    publish_identity_map_odom = LaunchConfiguration('publish_identity_map_odom')

    # 各个传感器的参数文件路径
    stm32_param_file = os.path.join(pkg_share, 'config', 'stm32_comm.yaml')
    lidar_param_file = os.path.join(pkg_share, 'config', 'sllidar_s2.yaml')
    imu_param_file = os.path.join(pkg_share, 'config', 'imu_h30.yaml')

    # 只有启用 STM32 且不用激光里程计时，STM32 节点才发布 odom -> base_link。
    stm32_with_tf = PythonExpression([
        '"', enable_stm32, '" == "true" and "', use_laser_odom, '" != "true"'
    ])
    # 激光里程计接管 odom -> base_link 时，STM32 只负责串口状态和 /cmd_vel 转发。
    stm32_without_tf = PythonExpression([
        '"', enable_stm32, '" == "true" and "', use_laser_odom, '" == "true"'
    ])
    # 不启用 STM32 且不用激光里程计时，才补临时 odom -> base_link。
    temporary_odom_base = PythonExpression([
        '"', enable_stm32, '" != "true" and "', use_laser_odom, '" != "true"'
    ])

    return LaunchDescription([
        # 声明可配置的 STM32 串口参数
        DeclareLaunchArgument(
            'stm32_port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            description='STM32 serial port'
        ),
        # 声明可配置的雷达串口参数
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
            'use_laser_odom',
            default_value='false',
            description='Use LiDAR-based odometry instead of STM32 odometry'
        ),
        DeclareLaunchArgument(
            'enable_stm32',
            default_value='true',
            description='Start STM32 base communication node'
        ),
        DeclareLaunchArgument(
            'stm32_read_only',
            default_value='false',
            description='Read STM32 odometry only; never send mode or velocity frames'
        ),
        DeclareLaunchArgument(
            'publish_identity_map_odom',
            default_value='false',
            description=(
                'Publish a temporary static map->odom transform for sensor-only '
                'tests. Keep false when running SLAM or Nav2.'
            )
        ),

        LogInfo(
            msg=(
                '[TF] publish_identity_map_odom=true: using temporary static '
                'map->odom for sensor-only testing.'
            ),
            condition=IfCondition(publish_identity_map_odom)
        ),

        # STM32 节点（发布 TF）—— 仅在 enable_stm32=true 且 use_laser_odom=false 时启用
        Node(
            package='stm32_nav_comm',
            executable='stm32_comm_node',
            name='stm32_comm_node',
            condition=IfCondition(stm32_with_tf),
            output='screen',
            parameters=[
                stm32_param_file,
                {
                    'stm32_serial_port': stm32_port,
                    'publish_tf': True,
                    'read_only': ParameterValue(stm32_read_only, value_type=bool),
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                }
            ]
        ),

        # STM32 节点（不发布 TF）—— 仅在 enable_stm32=true 且 use_laser_odom=true 时启用
        Node(
            package='stm32_nav_comm',
            executable='stm32_comm_node',
            name='stm32_comm_node',
            condition=IfCondition(stm32_without_tf),
            output='screen',
            parameters=[
                stm32_param_file,
                {
                    'stm32_serial_port': stm32_port,
                    'publish_tf': False,
                    'read_only': ParameterValue(stm32_read_only, value_type=bool),
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
                '--x', '0', '--y', '0', '--z', '-0.01805',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'gyro_link'
            ]
        ),

        # 静态坐标变换：base_link -> laser（雷达坐标系）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.65409',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser'
            ]
        ),

        # 临时静态坐标变换：odom -> base_link。
        # 仅用于不启动 STM32 的传感器单测；正式运行要由里程计发布动态 TF。
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='temporary_odom_to_base_tf',
            output='screen',
            condition=IfCondition(temporary_odom_base),
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'odom',
                '--child-frame-id', 'base_link'
            ]
        ),

        # 临时静态坐标变换：map -> odom。
        # 仅用于传感器/RViz 单测，正式建图由 slam_toolbox 发布，正式导航由 AMCL 发布。
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='temporary_map_to_odom_tf',
            output='screen',
            condition=IfCondition(publish_identity_map_odom),
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'map',
                '--child-frame-id', 'odom'
            ]
        ),
    ])
