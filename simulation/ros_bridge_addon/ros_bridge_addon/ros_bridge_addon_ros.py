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
from rclpy.logging import get_logger

from rclpy.node import Node
from ros_compatibility import ros_shutdown

#from std_msgs.msg import String
from custom_messages.msg import TrafficLightControl
from std_srvs.srv import Empty


from custom_messages.srv import CarlaTransform 
from std_srvs.srv import Empty


from carla import Client, TrafficLightState, Location, Rotation, Transform
from math import sqrt

class RosBridgeAddon(Node):

    def __init__(self):
        super().__init__('ros_bridge_addon', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.traffic_light_subscriber = self.create_subscription(TrafficLightControl, self.get_parameter('topics.subscriber.traffic_light').get_parameter_value().string_value, self.traffic_light_callback, 10)
        self.host = self.get_parameter('carla.host').get_parameter_value().string_value
        self.port = self.get_parameter('carla.port').get_parameter_value().integer_value
        self.find_service = self.create_service(Empty, 'carla/find_traffic_lights',
                self.find_traffic_lights_callback)
        self.spectator_set_transform = self.create_service(CarlaTransform, 'carla/spectator_set_transform',
                self.spectator_set_transform_callback)

        self.sphere_radius = self.get_parameter('service.radius').get_parameter_value().double_value

        try: 
            self.client = Client(self.host, self.port) 
            self.world = self.client.get_world() 
        except RuntimeError:
            self.get_logger().error(f'Connecting to host {self.host}:{self.port} failed, shutting down')
            ros_shutdown()
            del self.client
       
        self.get_logger().info(f'Carla client {self.host}:{self.port}')

    def traffic_light_callback(self, msg):
        for traffic_light_id,traffic_light_state in zip(msg.id_list,msg.status_list):
            actor = self.world.get_actor(traffic_light_id)
            try:
                actor.freeze(True)
                actor.set_state(TrafficLightState(traffic_light_state))
            except AttributeError:
                self.get_logger().error(f'Error setting the state for actor ID {traffic_light_id} of type {type(actor)}')

        self.get_logger().info('Received traffic light control: "%s"' % msg)
   
    def find_traffic_lights_callback(self, request, response):
        actors = self.world.get_actors()
        location = self.world.get_spectator().get_location()
        traffic_lights = [x for x in actors if 
                (x.type_id == 'traffic.traffic_light' and 
                    sqrt(
                        (x.get_location().x-location.x)**2+
                        (x.get_location().y-location.y)**2+
                        (x.get_location().z-location.z)**2 ) < self.sphere_radius)] 
        if traffic_lights:
            self.get_logger().info(f'Finding traffic lights in a radius of {self.sphere_radius}')
            [self.get_logger().info(f'ID: {x.id}, Position: {x.get_location().x}, {x.get_location().y}, {x.get_location().z}') for x in traffic_lights]
        return response

    def spectator_set_transform_callback(self, request, response):
        x,y,z,roll,pitch,yaw = request.x, request.y, request.z, request.roll, \
            request.pitch, request.yaw
        self.world.get_spectator().set_transform(Transform(Location(x=x,y=y,z=z), Rotation(roll=roll,
            pitch=pitch,yaw=yaw)))

        return response

    def __del__(self):
        del self.client
        del self.world

def main(args=None):
    rclpy.init(args=args)

    bridge = RosBridgeAddon()

    rclpy.spin(bridge)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
