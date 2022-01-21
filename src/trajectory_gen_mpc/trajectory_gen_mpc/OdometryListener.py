import rclpy
from rclpy import node
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance

class OdometryListener(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info("Odometry Node Initialized")
        self.odom = self.create_subscription(Odometry, "/carla/vehicle_00/odometry", self.odom_callback, 100)
        self.pose = PoseWithCovariance()
    def odom_callback(self, odometry):
        print("Got the message from odometery")
        pos_x = odometry.pose.pose.position.x
        pos_y = odometry.pose.pose.position.y
        pos_z = odometry.pose.pose.position.z
        print("Ego positon x: y: z:",pos_x, pos_y, pos_z)







def main(args = None):
    rclpy.init(args=args)
    odometry_listener = OdometryListener("odom_listener")
    rclpy.spin(odometry_listener)
    rclpy.shutdown()


if __name__ == "__main__":
    main()