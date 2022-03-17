import math
import rclpy
from rclpy.node import Node
from custom_messages.msg import VehicleStatus
from custom_messages.msg import VehicleControl
# import yaml


class DummyVehicle:
    def __init__(self, xpos, ypos, yaw, velocity, acceleration, steering):
        self.xpos = xpos # initial x postion [m]
        self.ypos = ypos # initial y postion [m]
        self.yaw = yaw # initial heading angle [rad]
        self.veloctiy = velocity # initial veloctiy [m/s]
        self.acceleration = acceleration # initial acceleration [m/s²]
        self.steering = steering # initial steering angle [rad]
        # vehicel_yaml_file = open("Config/Vehicleparameters.yaml")
        # parsed_yaml_veh = yaml.load(vehicel_yaml_file, Loader=yaml.FullLoader)
        # par_veh = parsed_yaml_veh["Vehicle"]
        # l = par_veh["wheelbase"]
        # l_h = par_veh["reardistance"]
        l = 3
        l_h = 1.5
        self.l = l # wheel base [m]
        self.l_h = l_h # distance from CoG to rear axle [m]

    def motion(self, dt):
        delta = self.steering
        x_v = self.xpos
        y_v = self.ypos
        l = self.l
        l_h = self.l_h
        # slip from steering angle
        beta = math.atan2(l_h * math.tan(delta), l)
        v_COG = self.veloctiy + self.acceleration * dt
        self.veloctiy = v_COG
        psi_v = self.yaw
        r = v_COG * (math.tan(delta) / l)
        xDot = v_COG * math.cos(psi_v)
        self.xpos = x_v + xDot * dt
        yDot = v_COG * math.sin(psi_v)
        self.ypos = y_v + yDot * dt
        psiDot = r
        self.yaw = psi_v + psiDot * dt
        if self.yaw > math.pi:
            self.yaw = self.yaw - 2*math.pi
        if self.yaw < -math.pi:
            self.yaw = self.yaw + 2*math.pi

class VehicleModel(Node):
    def __init__(self):
        super().__init__('dummy_vehicle')
        self.vehicle = DummyVehicle(0.0, 0.5, 0.0, 0.1, 0.0, 0.0)
        self._v_pos_publisher = self.create_publisher(VehicleStatus, '/aorta/altran/vehicle', 10)
        self._v_subscriber = self.create_subscription(VehicleControl, '/aorta/altran/trajectory_following_dummy',
                                                      self._subscribe_vehicle_control, 10)
        self.time = self.create_timer(0.01, self._publish_vehicle_motion)

    def _publish_vehicle_motion(self):
        self.vehicle.motion(0.01)
        msg_veh = VehicleStatus()
        msg_veh.x = self.vehicle.xpos
        msg_veh.y = self.vehicle.ypos
        msg_veh.yaw = self.vehicle.yaw
        msg_veh.curvature = 0.0
        msg_veh.velocity = self.vehicle.veloctiy
        msg_veh.acceleration = self.vehicle.acceleration
        self._v_pos_publisher.publish(msg_veh)

    def _subscribe_vehicle_control(self, controldata):
        self.vehicle.steering = controldata.steering
        self.vehicle.acceleration = controldata.acceleration
        # self.vehicle.motion(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleModel()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()