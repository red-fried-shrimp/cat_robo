import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
    LogInfo, RegisterEventHandler, SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """启动传感器、slam_toolbox、地图保存服务和 RViz。

    `map -> odom` 在这个启动文件中由 slam_toolbox 动态发布，所以传给
    sensors.launch.py 的 `publish_identity_map_odom` 必须保持 false，避免和
    slam_toolbox 同时发布同一条 TF。
    """
    pkg_share = get_package_share_directory('stm32_nav_bringup')
    slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    map_saver_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_saver_server',
        namespace='',
        name='map_saver_server',
        output='screen',
        parameters=[{'save_map_timeout': 5.0}],
    )

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
        DeclareLaunchArgument(
            'use_laser_odom', default_value='false',
            description=(
                'Experimental only: start the simple LaserScan odometry node. '
                'Keep false for scan-only SLAM.'
            )
        ),
        DeclareLaunchArgument(
            'enable_stm32', default_value='false',
            description='Scan-only SLAM safety default: do not start STM32 base communication node'
        ),
        DeclareLaunchArgument(
            'stm32_read_only', default_value='false',
            description='Read STM32 odometry without sending any control frames'
        ),

        # 强制 OpenGL 3.3，修复 Mesa RViz2 8-bit 地图纹理渲染 bug
        SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'sensors.launch.py')
            ),
            launch_arguments={
                'stm32_port': LaunchConfiguration('stm32_port'),
                'lidar_port': LaunchConfiguration('lidar_port'),
                'use_laser_odom': LaunchConfiguration('use_laser_odom'),
                'enable_stm32': LaunchConfiguration('enable_stm32'),
                'stm32_read_only': LaunchConfiguration('stm32_read_only'),
                'publish_identity_map_odom': 'false',
            }.items()
        ),

        # 简易 LiDAR 里程计（仅调试时 use_laser_odom=true 才启动）。
        # 默认关闭，让 slam_toolbox 直接根据 /scan 做 scan matching。
        Node(
            package='stm32_nav_bringup',
            executable='laser_odometry.py',
            name='laser_odometry',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_laser_odom'))
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params]
        ),

        map_saver_server,

        RegisterEventHandler(
            OnProcessStart(
                target_action=map_saver_server,
                on_start=[
                    LogInfo(msg='[map_saver] Configuring...'),
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=lambda action: action == map_saver_server,
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )),
                ],
            ),
        ),

        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=map_saver_server,
                goal_state='inactive',
                entities=[
                    LogInfo(msg='[map_saver] Activating...'),
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=lambda action: action == map_saver_server,
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            ),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])
