from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stm32_nav_comm',
            executable='stm32_comm_node',
            name='stm32_comm_node',
            output='screen',
            parameters=[{
                'stm32_serial_port': '/dev/ttyUSB0',
                'stm32_baud_rate': 115200,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_tf': True,
                'wheel_radius': 0.042,
                'wheel_base': 0.1977,
                'cmd_timeout_sec': 0.5,
                'odom_timeout_sec': 0.5,
            }]
        )
    ])

