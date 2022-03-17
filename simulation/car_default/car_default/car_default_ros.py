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
from ros_compatibility import QoSProfile

from custom_messages.msg import Trajectory
from nav_msgs.msg import Path
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from carla_msgs.msg import CarlaEgoVehicleControl

from . import utils 
from numpy.random import choice

class CarDefault(Node):

    def __init__(self):
        super().__init__('car_default', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        

        self.control_publisher = self.create_publisher(CarlaEgoVehicleControl, self.get_parameter('topics.publisher.control').get_parameter_value().string_value, 10)
        control_timer_period = 1.0
        self.misbehave = self.get_parameter('misbehave').get_parameter_value().bool_value
        misbehave_params = self.get_parameters_by_prefix('misbehave_params')
        self.misbehave_types = [x.split(".")[0] for x in misbehave_params if x.endswith(".probability")]
        self.get_logger().info(f'misbehave_types: {self.misbehave_types}')
        self.misbehave_probabilities = [self.get_parameter(f'misbehave_params.{t}.probability').get_parameter_value().double_value for t in self.misbehave_types]
        self.get_logger().info(f'misbehave_probabilities: {self.misbehave_probabilities}')

        self.control_timer = self.create_timer(control_timer_period, self.control_callback)

        self.trajectory_subscriber = self.create_subscription(Trajectory, self.get_parameter('topics.subscriber.trajectory').get_parameter_value().string_value, self.trajectory_callback, 10)

        self.waypoint_publisher = self.create_publisher(Path, self.get_parameter('topics.publisher.waypoint').get_parameter_value().string_value, QoSProfile(depth=1, durability=True))
        self.waypoint_speed_publisher = self.create_publisher(Float64, self.get_parameter('topics.publisher.waypoint_speed').get_parameter_value().string_value, QoSProfile(depth=1, durability=True))

        self.trajectory = None
        self.speed = self.get_parameter('car.speed').get_parameter_value().double_value

    def control_callback(self):
        msg = CarlaEgoVehicleControl()
        self.control_publisher.publish(msg)
        self.get_logger().info('Publishing Ego Vehicle Control')

    def trajectory_callback(self, msg):
        self.get_logger().info('Received Trajectory: "%s"' % msg)
        self.trajectory = msg

        if self.misbehave:
            misbehave_type_selection = choice(self.misbehave_types, p=self.misbehave_probabilities)
            self.get_logger().info(f'Node is going to execute a mishaviour of type {misbehave_type_selection}')
            try:
                misbehave_function = getattr(utils, misbehave_type_selection)
                misbehave_params = self.get_parameter(f'misbehave_params.{misbehave_type_selection}.params').get_parameter_value().double_array_value
                misbehave_function(self, misbehave_params)
            except AttributeError:
                self.get_logger().warn(f'{misbehave_type_selection} is not implemented.')

       
        # These calls have to be after the misbehave_function call
        self.waypoints = self.get_path_from_trajectory(self.trajectory)
        self.waypoint_speed = self.get_speed_from_trajectory(self.trajectory)

        self.waypoint_speed_callback()
        self.waypoint_callback()
    
    def waypoint_callback(self):
        self.waypoint_publisher.publish(self.waypoints)
        self.get_logger().info('Publishing waypoints.')

    def waypoint_speed_callback(self):
        self.waypoint_speed_publisher.publish(self.waypoint_speed)
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

    car = CarDefault()

    rclpy.spin(car)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    car.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
