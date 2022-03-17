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

from std_msgs.msg import Bool
from carla_msgs.msg import CarlaEgoVehicleControl


class TukFilter(Node):

    def __init__(self):
        super().__init__('tuk_filter')
        self.control = self.create_publisher(CarlaEgoVehicleControl, '/carla/altran/vehicle_control_cmd', 10)
        control_timer_period = 0.5  # seconds
        self.control_timer = self.create_timer(control_timer_period, self.control_callback)

        self.control_bool = self.create_publisher(Bool, '/carla/altran/vehicle_control_manual_override', 10)
        self.bool_timer = self.create_timer(0.5, self.bool_callback)


 

    def control_callback(self):
        msg = CarlaEgoVehicleControl()
        msg.throttle = 1.0
        msg.header.stamp = self.get_clock().now().to_msg()
        self.control.publish(msg)


    def bool_callback(self):
        msg = Bool()
        msg.data = True
        self.control_bool.publish(msg)





def main(args=None):
    rclpy.init(args=args)

    filter = TukFilter()

    rclpy.spin(filter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
