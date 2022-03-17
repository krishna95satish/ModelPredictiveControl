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

from custom_messages.msg import AkkaSensor


class AkkaSensorDummy(Node):

    def __init__(self):
        super().__init__('tof_sensor_dummy', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.akka_tof_sensor = self.create_publisher(AkkaSensor, self.get_parameter('topics.publisher.akka').get_parameter_value().string_value, 10)
        self.tuk_tof_sensor = self.create_publisher(AkkaSensor, self.get_parameter('topics.publisher.tuk').get_parameter_value().string_value, 10)

    def akka_sensor_akka_callback(self):
        msg = AkkaSensor()
        self.get_logger().info('Publishing Akka Sensor Data: "%s"' % msg)


    def tuk_sensor_akka_callback(self):
        msg = AkkaSensor()
        self.get_logger().info('Publishing Akka Sensor Data: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)

    akkaSensor = AkkaSensorDummy()

    rclpy.spin(akkaSensor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    edge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
