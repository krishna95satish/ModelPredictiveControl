import rclpy
from rclpy.node import Node
from custom_messages.msg import VehicleControlParameter


class ControlParameter(Node):
    def __init__(self):
        super().__init__('trajectory_following_parameter', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self._v_par_publisher = self.create_publisher(VehicleControlParameter, '/carla/altran/following_control_parameters', 10)
        self.time = self.create_timer(1, self._publish_control_parameter)

    def _publish_control_parameter(self):
        msg_par = VehicleControlParameter()
        msg_par.qrg_v = float(self.get_parameter('car_control.qrg_v').get_parameter_value().double_value)
        msg_par.qrg_bfryt = float(self.get_parameter('car_control.qrg_bfryt').get_parameter_value().double_value)
        msg_par.qrg_bfrydt = float(self.get_parameter('car_control.qrg_bfrydt').get_parameter_value().double_value)
        msg_par.qrg_bfryit = float(self.get_parameter('car_control.qrg_bfryit').get_parameter_value().double_value)
        msg_par.qrg_ff = float(self.get_parameter('car_control.qrg_ff').get_parameter_value().double_value)
        msg_par.qrg_sgs = float(self.get_parameter('car_control.qrg_sgs').get_parameter_value().double_value)
        msg_par.lrg_bfrt = float(self.get_parameter('car_control.lrg_bfrt').get_parameter_value().double_value)
        msg_par.lrg_bfrit = float(self.get_parameter('car_control.lrg_bfrit').get_parameter_value().double_value)
        msg_par.lrg_bfrxt = float(self.get_parameter('car_control.lrg_bfrxt').get_parameter_value().double_value)
        msg_par.lrg_bfrxdt = float(self.get_parameter('car_control.lrg_bfrxdt').get_parameter_value().double_value)
        msg_par.lrg_bfrxit = float(self.get_parameter('car_control.lrg_bfrxit').get_parameter_value().double_value)
        self._v_par_publisher.publish(msg_par)


def main(args=None):
    rclpy.init(args=args)
    node = ControlParameter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()