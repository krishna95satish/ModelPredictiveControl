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

#from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from custom_messages.msg import TrajectoryPoint, Trajectory, ObjectList, TrafficLightProgram
from carla_msgs.msg import CarlaTrafficLightStatusList


class AortaServer(Node):

    def __init__(self):
        super().__init__('aorta_server', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        # Initialize all subscriber

        self.altran_gnss_subscriber = self.create_subscription(NavSatFix, self.get_parameter('topics.subscriber.gnss.altran').get_parameter_value().string_value, self.gnss_callback, 10)
        self.altran_objects_subscriber = self.create_subscription(ObjectList, self.get_parameter('topics.subscriber.object_list.altran').get_parameter_value().string_value, self.objects_callback, 10)
        self.akka_gnss_subscriber = self.create_subscription(NavSatFix, self.get_parameter('topics.subscriber.gnss.akka').get_parameter_value().string_value, self.gnss_callback, 10)
        self.akka_objects_subscriber = self.create_subscription(ObjectList, self.get_parameter('topics.subscriber.object_list.akka').get_parameter_value().string_value, self.objects_callback, 10)
        self.tuk_gnss_subscriber = self.create_subscription(NavSatFix, self.get_parameter('topics.subscriber.gnss.tuk').get_parameter_value().string_value, self.gnss_callback, 10)
        self.tuk_objects_subscriber = self.create_subscription(ObjectList, self.get_parameter('topics.subscriber.object_list.tuk').get_parameter_value().string_value, self.objects_callback, 10)
        self.edge_subscriber = self.create_subscription(ObjectList, self.get_parameter('topics.subscriber.edge').get_parameter_value().string_value, self.edge_callback, 10)
        self.traffic_light_subscriber = self.create_subscription(CarlaTrafficLightStatusList, self.get_parameter('topics.subscriber.traffic_light').get_parameter_value().string_value, self.traffic_light_callback, 10)


        # Initialize all Publisher
        self.traffic_light_publisher = self.create_publisher(TrafficLightProgram, self.get_parameter('topics.publisher.traffic_light').get_parameter_value().string_value, 10)

        # Load dummy trajectories
        self.trajectories = self.get_parameters_by_prefix("trajectory")
        for key in self.trajectories:
            # First we create the publisher
            if self.get_parameter(f'topics.publisher.trajectory.{key}').get_parameter_value().string_value:
                setattr(self,f'{key}_trajectory_publisher',self.create_publisher(Trajectory, 
                  self.get_parameter(f'topics.publisher.trajectory.{key}').get_parameter_value().string_value, 10))
            else:
                self.get_logger().warn(f'No trajectory publisher topic defined for trajectory {key}. Please define a publisher topic in the configuration file.')
                continue

            # Load the trajectory from the yaml file
            tr = self.trajectories[key].get_parameter_value().double_array_value
            
            # Iterate over the trajectory as a pair of (x,y)
            setattr(self, f'{key}_trajectory_dummy', Trajectory()) 
            for x,y in zip(tr[::2], tr[1::2]):
                point = TrajectoryPoint()
                point.x = x
                point.y = y

                # Append the point and set the header stamp
                getattr(self, f'{key}_trajectory_dummy').points.append(point)
                getattr(self, f'{key}_trajectory_dummy').header.stamp = self.get_clock().now().to_msg()

        # Send dummy data in a loop
        resend_dummy_timer = 60.0

        self.trajectory_timer = self.create_timer(resend_dummy_timer, self.timer_callback)
        #self.traffic_light_timer = self.create_timer(resend_dummy_timer, self.traffic_light_program_callback)


    # Subscriber callbacks

    def gnss_callback(self, msg):
        self.get_logger().info('Received GNSS: "%s"' % msg)

    def objects_callback(self, msg):
        self.get_logger().info('Received ObjectList: "%s"' % msg)

    def edge_callback(self, msg):
        self.get_logger().info('Received Edge-ObjectList: "%s"' % msg)

    def traffic_light_callback(self, msg):
        self.get_logger().info('Received Traffic Light Message: "%s"' % msg)

    # Trajectory callbacks
    def timer_callback(self):
        self.get_logger().info(f'Publishing trajectories')

        # Iterate over all trajectories defined in the yaml file
        for key in self.trajectories:
            try:
                # Get the trajectory from the class
                msg = getattr(self,f'{key}_trajectory_dummy')
                msg.header.stamp = self.get_clock().now().to_msg()
                # Get the publisher and publish
                getattr(self,f'{key}_trajectory_publisher').publish(msg)
            except AttributeError as e:
                self.get_logger().warn(f'{e}')
                continue

    # Additional callbacks
    def traffic_light_program_callback(self):
        msg = TrafficLightProgram()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.traffic_light_publisher.publish(msg)
        self.get_logger().info('Sending Traffic Light Program Message: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)

    server = AortaServer()

    rclpy.spin(server)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

