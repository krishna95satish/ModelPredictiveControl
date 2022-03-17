# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from ros_compatibility import QoSProfile, CompatibleNode, loginfo, ros_init, ROS_VERSION

#from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, CameraInfo, Image, PointCloud2
from custom_messages.msg import TrajectoryPoint, Trajectory, Object, ObjectList, AkkaSensor
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Pose
from carla_msgs.msg import CarlaEgoVehicleStatus, CarlaEgoVehicleInfo, CarlaEgoVehicleControl



class CarAkka(Node):

    def __init__(self):
        super().__init__('car_akka', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        # Initialize all subscriber
        
        self.trajectory_subscriber = self.create_subscription(Trajectory, self.get_parameter('topics.subscriber.trajectory').get_parameter_value().string_value, self.trajectory_callback, 10)
        self.image_subscriber = self.create_subscription(Image, self.get_parameter('topics.subscriber.image').get_parameter_value().string_value, self.image_callback, 10)
        self.camera_info_subscriber = self.create_subscription(CameraInfo, self.get_parameter('topics.subscriber.camera_info').get_parameter_value().string_value, self.camera_info_callback, 10)
        self.lidar_subscriber = self.create_subscription(PointCloud2, self.get_parameter('topics.subscriber.lidar').get_parameter_value().string_value, self.lidar_callback, 10)
        self.gnss_subscriber = self.create_subscription(NavSatFix, self.get_parameter('topics.subscriber.gnss').get_parameter_value().string_value, self.carla_gnss_callback, 10)
        self.odometry_subscriber = self.create_subscription(Odometry, self.get_parameter('topics.subscriber.odometry').get_parameter_value().string_value, self.odometry_callback, 10)
        self.ego_vehicle_subscriber = self.create_subscription(CarlaEgoVehicleStatus, self.get_parameter('topics.subscriber.vehicle_status').get_parameter_value().string_value, self.ego_vehicle_callback, 10)
        self.ego_wheel_subscriber = self.create_subscription(CarlaEgoVehicleInfo, self.get_parameter('topics.subscriber.vehicle_info').get_parameter_value().string_value, self.ego_wheels_callback, 10)
        self.akka_sensor_subscriber = self.create_subscription(AkkaSensor, self.get_parameter('topics.subscriber.tof_sensor').get_parameter_value().string_value, self.akka_sensor_callback, 10)


        # Initialize all publisher

        self.gnss_publisher = self.create_publisher(NavSatFix, self.get_parameter('topics.publisher.gnss').get_parameter_value().string_value, 10)
        self.object_publisher = self.create_publisher(ObjectList, self.get_parameter('topics.publisher.objects').get_parameter_value().string_value, 10)
        self.control_publisher = self.create_publisher(CarlaEgoVehicleControl, self.get_parameter('topics.publisher.control').get_parameter_value().string_value, 10)
        self.waypoint_publisher = self.create_publisher(Path, self.get_parameter('topics.publisher.waypoint').get_parameter_value().string_value, QoSProfile(depth=1, durability=True))
        self.waypoint_speed_publisher = self.create_publisher(Float64, self.get_parameter('topics.publisher.waypoint_speed').get_parameter_value().string_value, QoSProfile(depth=1, durability=True))


        self.trajectory = None
        self.speed = 10.0



    # Subscriber callbacks

    def trajectory_callback(self, msg):
        self.get_logger().info('Received Trajectory: "%s"' % msg)
        self.trajectory = msg
        self.waypoint_speed_callback()
        self.waypoint_callback()

    def image_callback(self, msg):
        self.get_logger().info('Received image.')

    def camera_info_callback(self, msg):
        self.get_logger().info('Received camera info.')

    def lidar_callback(self, msg):
        self.get_logger().info('Received lidar.')

    def carla_gnss_callback(self, msg):
        self.get_logger().info('Received gnss.')

    def odometry_callback(self, msg):
        self.get_logger().info('Received odometry.')

    def ego_vehicle_callback(self, msg):
        self.get_logger().info('Received ego vehicle.')

    def ego_wheels_callback(self, msg):
        self.get_logger().info('Received ego wheels.')

    def akka_sensor_callback(self, msg):
        self.get_logger().info('Received akka sensor.')

    # Publisher callbacks

    def gnss_callback(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.gnss_publisher.publish(msg)
        self.get_logger().info('Publishing GNSS: "%s"' % msg)

    def object_callback(self):
        msg = ObjectList()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.object_publisher.publish(msg)
        self.get_logger().info('Publishing ObjectList: "%s"' % msg)

    def control_callback(self):
        msg = CarlaEgoVehicleControl()
        self.control_publisher.publish(msg)
        self.get_logger().info('Publishing Ego Vehicle Control')

    def waypoint_callback(self):
        msg = self.get_path_from_trajectory(self.trajectory)
        self.waypoint_publisher.publish(msg)
        self.get_logger().info('Publishing waypoints.')

    def waypoint_speed_callback(self):
        msg = self.get_speed_from_trajectory(self.trajectory)
        self.waypoint_speed_publisher.publish(msg)
        self.get_logger().info('Publishing waypoint speed.')


    # Helper functions
    
    def get_path_from_trajectory(self, trajectory):
        # TODO: also use other values
        path = Path()
        for point in trajectory.points:
            pose = PoseStamped()
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = 0.0

            path.poses.append(pose)

        path.header = trajectory.header

        return path

    def get_speed_from_trajectory(self, trajectory):
        # TODO: real implementation
        speed = Float64()
        speed.data = self.speed

        return speed


def main(args=None):
    rclpy.init(args=args)

    car = CarAkka()

    rclpy.spin(car)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    car.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
