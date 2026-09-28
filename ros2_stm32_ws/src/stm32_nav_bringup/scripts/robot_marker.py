#!/usr/bin/python3

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker


class RobotMarker(Node):
    def __init__(self):
        super().__init__('robot_marker')

        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('topic', '/robot_marker')
        self.declare_parameter('length', 0.42)
        self.declare_parameter('width', 0.30)
        self.declare_parameter('height', 0.08)
        self.declare_parameter('line_width', 0.035)

        self.frame_id = self.get_parameter('frame_id').value
        topic = self.get_parameter('topic').value
        self.length = float(self.get_parameter('length').value)
        self.width = float(self.get_parameter('width').value)
        self.height = float(self.get_parameter('height').value)
        self.line_width = float(self.get_parameter('line_width').value)

        self.publisher = self.create_publisher(Marker, topic, 1)
        self.timer = self.create_timer(0.2, self.publish_marker)

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'robot_footprint'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.line_width
        marker.color.r = 0.0
        marker.color.g = 0.85
        marker.color.b = 1.0
        marker.color.a = 1.0

        front_x = self.length * 0.5
        rear_x = -self.length * 0.5
        half_y = self.width * 0.5
        z = self.height

        marker.points = [
            Point(x=front_x, y=0.0, z=z),
            Point(x=rear_x, y=half_y, z=z),
            Point(x=rear_x, y=-half_y, z=z),
            Point(x=front_x, y=0.0, z=z),
        ]
        self.publisher.publish(marker)


def main():
    rclpy.init()
    node = RobotMarker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
