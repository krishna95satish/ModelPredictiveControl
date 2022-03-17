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
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from ros_compatibility import QoSProfile
import time

#from std_msgs.msg import String


class Demo_01(Node):

    def __init__(self):
        super().__init__('aorta_demo', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        # Initialize all Publisher
        self.trajectories = self.get_parameters_by_prefix("trajectory")
        self.speeds = self.get_parameters_by_prefix("speed")
        for key in self.trajectories:
            setattr(self,key,self.create_publisher(Path, f"/carla/{key}/waypoints", QoSProfile(depth=1, durability=True)))
            setattr(self,key+"_speed",self.create_publisher(Float64, f"/carla/{key}/speed_command", QoSProfile(depth=1, durability=True)))
            tr = self.trajectories[key].get_parameter_value().double_array_value
            setattr(self, key+"_trajectory", Path())
            for x,y in zip(tr[::2], tr[1::2]):
                point = PoseStamped()
                point.pose.position.x = x
                point.pose.position.y = y
                point.pose.position.z = 0.0
                getattr(self, key+"_trajectory").poses.append(point)
                getattr(self, key+"_trajectory").header.stamp = self.get_clock().now().to_msg()


       # Send dummy data in a loop
        resend_dummy_timer = 30.0

        self.timer = self.create_timer(resend_dummy_timer, self.timer_callback)


    # Trajectory callbacks

    def timer_callback(self):
        self.get_logger().info(f"Publishing trajectories")
        for key in self.trajectories:
            msg = getattr(self,key+"_trajectory")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = key
            speed = Float64()
            speed.data = self.speeds[key].get_parameter_value().double_value
            getattr(self,key+"_speed").publish(speed)
            getattr(self,key).publish(msg)


def main(args=None):
    rclpy.init(args=args)

    server = Demo_01()

    rclpy.spin(server)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
