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
from custom_messages.msg import TrajectoryPoint, Trajectory, Object, ObjectList

from time import sleep 
import random

class AkkaFilter(Node):

    def __init__(self):
        super().__init__('akka_filter', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.gnss_publisher = self.create_publisher(NavSatFix, self.get_parameter('topics.publisher.gnss').get_parameter_value().string_value, 10)
        self.trajectory_publisher = self.create_publisher(Trajectory, self.get_parameter('topics.publisher.trajectory').get_parameter_value().string_value, 10)
        self.objects_publisher = self.create_publisher(ObjectList, self.get_parameter('topics.publisher.objects').get_parameter_value().string_value, 10)

        self.trajectory_subscriber = self.create_subscription(Trajectory, self.get_parameter('topics.subscriber.trajectory').get_parameter_value().string_value, lambda x: self.filter_callback(x, 'trajectory'), 10)
        self.gnss_subscriber = self.create_subscription(NavSatFix, self.get_parameter('topics.subscriber.gnss').get_parameter_value().string_value, lambda x: self.filter_callback(x, 'gnss'), 10)
        self.objects_subscriber = self.create_subscription(ObjectList, self.get_parameter('topics.subscriber.objects').get_parameter_value().string_value, lambda x: self.filter_callback(x, 'objects'), 10)

        self.communication_parameters = self.get_parameters_by_prefix("communication")
        self.comm_param_keys = [x.split(".")[0] for x in self.communication_parameters]

    def filter_callback(self, msg, topic=None):
        if topic is not None:
            publisher_name = f"{topic}_publisher"

            if topic in self.comm_param_keys:
                p_drop = self.communication_parameters[f'{topic}.drop_probability'].get_parameter_value().double_value

                if p_drop>random.random():
                    self.get_logger().info(f'{topic} message dropped: {msg}')
                    msg = None
                
                if msg is not None:
                    p_delay = self.communication_parameters[f'{topic}.delay_probability'].get_parameter_value().double_value
                    
                    if p_delay is not None and p_delay<random.random():
                        # Message not delayed
                        getattr(self, publisher_name).publish(msg)
                    else:
                        # Message delayed
                        delay = self.communication_parameters[f'{topic}.delay_time_ms'].get_parameter_value().double_value
                        self.get_logger().info(f'{topic} message delayed for {delay}ms: {msg}')
                        sleep(delay*0.001)
                        getattr(self, publisher_name).publish(msg)

            else:
                getattr(self, publisher_name).publish(msg)


def main(args=None):
    rclpy.init(args=args)

    filter = AkkaFilter()

    rclpy.spin(filter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
