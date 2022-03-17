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
from custom_messages.msg import TrajectoryPoint, Trajectory, Object, ObjectList
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Pose
from carla_msgs.msg import CarlaEgoVehicleStatus, CarlaEgoVehicleInfoWheel, CarlaEgoVehicleControl


class CarEmergency(Node):

    def __init__(self):
        super().__init__('car_emergency', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        # Initialize all subscriber

        self.trajectory_subscriber = self.create_subscription(Trajectory, self.get_parameter('topics.subscriber.trajectory').get_parameter_value().string_value, self.trajectory_callback, 10)

        # Initialize alle pubilsher
        self.waypoint_publisher = self.create_publisher(Path, self.get_parameter('topics.publisher.waypoint').get_parameter_value().string_value, QoSProfile(depth=1, durability=True))
        self.waypoint_speed_publisher = self.create_publisher(Float64, self.get_parameter('topics.publisher.waypoint_speed').get_parameter_value().string_value, QoSProfile(depth=1, durability=True))


        self.trajectory = None
        self.speed = 30.0

    # Subscriber callback

    def trajectory_callback(self, msg):
        self.get_logger().info('Received Trajectory: "%s"' % msg)
        self.trajectory = msg
        self.waypoint_speed_callback()
        self.waypoint_callback()

    # Publisher callback

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

    car = CarEmergency()

    rclpy.spin(car)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    car.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
