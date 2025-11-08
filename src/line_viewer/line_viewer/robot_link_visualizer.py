#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class SimpleLinePublisher(Node):
    def __init__(self):
        super().__init__('simple_line_publisher')
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(1.0, self.publish_line)
        self.get_logger().info('Line Publisher Node Started')

    def publish_line(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "my_lines"
        marker.id = 0
        marker.type = Marker.LINE_STRIP # Or Marker.LINE_LIST
        marker.action = Marker.ADD

        # Set line properties
        marker.scale.x = 0.05 # Line width
        marker.color.a = 1.0 # Alpha (transparency)
        marker.color.r = 1.0 # Red
        marker.color.g = 0.0 # Green
        marker.color.b = 0.0 # Blue

        # Define points for the line
        point1 = Point()
        point1.x = 0.0
        point1.y = 0.0
        point1.z = 0.0
        marker.points.append(point1)

        point2 = Point()
        point2.x = 1.0
        point2.y = 1.0
        point2.z = 0.0
        marker.points.append(point2)

        point3 = Point()
        point3.x = 1.0
        point3.y = 0.0
        point3.z = 0.0
        marker.points.append(point3)

        self.marker_publisher.publish(marker)
        self.get_logger().info('Published line marker')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleLinePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
