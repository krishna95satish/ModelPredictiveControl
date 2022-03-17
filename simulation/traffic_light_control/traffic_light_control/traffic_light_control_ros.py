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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

#from std_msgs.msg import String
from custom_messages.msg import TrafficLightControl, TrafficLightProgram

import json
from itertools import cycle

class TrafficLightController(Node):

    def __init__(self):
        super().__init__('traffic_light_control', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.timer = None 
        self.states = None
        self.time_stamps = None
        self.group = MutuallyExclusiveCallbackGroup()
        self.programs = self.get_parameters_by_prefix('programs')
        
        self.traffic_light_subscriber = self.create_subscription(TrafficLightProgram, self.get_parameter('topics.subscriber.program').get_parameter_value().string_value, self.traffic_light_callback, 10)
        self.traffic_light_control_publisher = self.create_publisher(TrafficLightControl, self.get_parameter('topics.publisher.control').get_parameter_value().string_value, 10)

    def traffic_light_callback(self, msg):

        if str(msg.program_id) in self.programs.keys():
            self.get_logger().info('Received traffic light program: "%s"' % msg)
            
            program = [json.loads(x) for x in 
                    self.programs[str(msg.program_id)].get_parameter_value().string_array_value]
            self.time_stamps = cycle([x["time"] for x in program])
            self.states = cycle([(x["id"], x["state"]) for x in program])

            if self.timer is not None:
                self.destroy_timer(self.timer)

            self.timer = self.create_timer(next(self.time_stamps), self.timer_callback, self.group)

        else:
            self.get_logger().info(f'Traffic light program with id {msg.program_id} is not available')
            if self.timer is not None:
                self.destroy_timer(self.timer)

    def timer_callback(self):
        self.destroy_timer(self.timer)

        tl_msg = TrafficLightControl()
        tl_msg.header.stamp = self.get_clock().now().to_msg()
        tl_msg.id_list, tl_msg.status_list = next(self.states) 

        self.traffic_light_control_publisher.publish(tl_msg)
        self.get_logger().info('Publishing traffic light control: "%s"' % tl_msg)

        self.timer = self.create_timer(next(self.time_stamps), self.timer_callback, self.group)


def main(args=None):
    rclpy.init(args=args)

    control = TrafficLightController()

    rclpy.spin(control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
