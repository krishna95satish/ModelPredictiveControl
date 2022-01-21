from numpy.core.fromnumeric import swapaxes
import rclpy
import numpy as np
from casadi import pi
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rclpy.duration import Duration


class VizNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.viz_publisher = self.create_publisher(Marker, "my_frame", 1000)
        self.timer = self.create_timer(1, self.publisher_cb)
        self.temp_x = 1.0
        self.temp_y = 1.0
        self.marker = Marker()
    
    def publisher_cb(self):
        self.marker.header.frame_id = "my_frame"
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.ns = "my_frame"
        self.marker.action = Marker.ADD
        self.marker.id = 1
        self.marker.type = Marker.LINE_STRIP
        self.marker.scale.x = 0.2
        self.marker.color.r = 1.0
        self.marker.color.a = 1.0
        self.point = Point()
        self.point.x = self.temp_x
        self.point.y = self.temp_y
        self.point.z = 0.0
        self.marker.pose.orientation.w = np.sin(pi)
        self.marker.lifetime = Duration(seconds=300, nanoseconds=5e2).to_msg()
        self.marker.points.append(self.point)
        self.temp_x += 1
        self.temp_y += 0
        self.viz_publisher.publish(self.marker)
        


def main(args=None):
    rclpy.init(args=args)
    viz_node = VizNode("test_viz_node")
    rclpy.spin(viz_node)
    rclpy.shutdown()




if __name__ == "__main__":
    main()