from nav_msgs import msg
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus


class OdometryListener(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info("Odometry Node Initialized")
        self.odom = self.create_subscription(
            Odometry, "/carla/vehicle_00/odometry", self.odom_callback, 100)
        self.dummy_control_publisher = self.create_publisher(
            CarlaEgoVehicleControl, '/carla/vehicle_00/vehicle_control_cmd', 10)
        self.velocity_subscriber = self.create_subscription(CarlaEgoVehicleStatus, "/carla/vehicle_00/vehicle_status", self.velocity_callback, 10)
        self.pose = PoseWithCovariance()
        self.ego_vehicle_control = CarlaEgoVehicleControl()
    
    def velocity_callback(self, velocity):
        self.velocity = velocity.velocity

    def odom_callback(self, odometry):
        print("Got the message from odometery")
        pos_x = odometry.pose.pose.position.x
        pos_y = odometry.pose.pose.position.y
        pos_z = odometry.pose.pose.position.z
        print("Ego positon x: y: v:", pos_x, pos_y, self.velocity)
        if (pos_x < -120):
            print("Sending control commands to carla")
            self.ego_vehicle_control.throttle = 0.1
            self.ego_vehicle_control.steer = 0.00
            self.ego_vehicle_control.gear = 1
        else :
            self.ego_vehicle_control.brake = 1.0

        self.dummy_control_publisher.publish(self.ego_vehicle_control)


def main(args=None):
    rclpy.init(args=args)
    odometry_listener = OdometryListener("odom_listener")
    rclpy.spin(odometry_listener)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
