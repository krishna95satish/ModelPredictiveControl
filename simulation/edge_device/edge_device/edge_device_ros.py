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
from custom_messages.msg import Object, ObjectList
from sensor_msgs.msg import CameraInfo, Image


class Edge(Node):

    def __init__(self):
        super().__init__('edge_device', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.object_publisher = self.create_publisher(ObjectList, self.get_parameter('topics.publisher.objects').get_parameter_value().string_value, 10)

        self.image_subscriber = self.create_subscription(Image, self.get_parameter('topics.subscriber.image').get_parameter_value().string_value, self.image_callback, 10)
        self.camera_info_subscriber = self.create_subscription(CameraInfo, self.get_parameter('topics.subscriber.camera_info').get_parameter_value().string_value, self.camera_info_callback, 10)

    def object_callback(self):
        msg = ObjectList()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.object_publisher.publish(msg)
        self.get_logger().info('Publishing ObjectList: "%s"' % msg)

    def image_callback(self, msg):
        self.get_logger().info('Received image.')

    def camera_info_callback(self, msg):
        self.get_logger().info('Received camera info.')


def main(args=None):
    rclpy.init(args=args)

    edge = Edge()

    rclpy.spin(edge)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    edge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
