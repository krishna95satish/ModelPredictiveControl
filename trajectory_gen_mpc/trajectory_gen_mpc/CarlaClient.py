#!/usr/bin/env python3

from threading import Thread
import rclpy
from rclpy.node import Node
from casadi import pi, inf
import casadi as cd
from custom_interfaces.srv import Trajectory
from trajectory_gen_mpc.GlobalConstraints import VehicleConstraints
from functools import partial
from trajectory_gen_mpc.NonHolonomic import NonHolonomic


class CarlaClient(Node):
    def __init__(self, nodeName):
        super().__init__(nodeName)
        # We do not need to create a vehicle object here since we need to extract the pose from the carla simulator not from the mortion model,
        #  This is just for testing
        vehicle_constrinats = VehicleConstraints(
            0.5, 0.1, -0.1, (pi/6), -(pi/6), inf, -inf, 5.0, -5.0, inf, -inf, inf, -inf, 0.3)
        car = NonHolonomic(vehicle_constrinats)
        # expects states and constrol action to produce the nonlinear term
        car.create_model()
        self.non_linear_fun_generator = car.get_model()
        ########################################################## Code above is only for testing  ###############################

        self.client = self.create_client(Trajectory, "TrajectoryServer")
        while not self.client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for the server to join")

        self.request = Trajectory.Request()
        self.request.initial_state_x = 3.0
        self.request.initial_state_y = -3.0
        self.request.initial_state_v = 0.0
        self.request.initial_state_theta = pi/2

        self.request.final_state_x = 2.7
        self.request.final_state_y = -1.7
        self.request.final_state_v = 0.0
        self.request.final_state_theta = pi/2

        self.mpc_iter = 0
        self.delta_T = 0.3
        self.sim_time = 20
        self.error_tolerance = 0.105
        self.current_state = cd.DM([self.request.initial_state_x,
                                    self.request.initial_state_y,
                                    self.request.initial_state_v,
                                    self.request.initial_state_theta]
                                   )
        self.target_state = cd.DM([self.request.final_state_x,
                                   self.request.final_state_v,
                                   self.request.final_state_y,
                                   self.request.final_state_theta]
                                 )
        self.simulation_loop()

    def simulation_loop(self):

        while (cd.norm_2(self.current_state - self.target_state) > self.error_tolerance) and (self.mpc_iter * self.delta_T < self.sim_time):
            self.clientCb()
            self.mpc_iter = self.mpc_iter + 1

    def clientCb(self):
        state_init = cd.DM([self.request.initial_state_x, 
                            self.request.initial_state_y, 
                            self.request.initial_state_v, 
                            self.request.initial_state_theta]
                            )
        print(state_init)
        future = self.client.call_async(self.request)
        future.add_done_callback(partial(self.futureDoneCb))

    def futureDoneCb(self, future):
        try:
            response = future.result()
            self.get_logger().info("The Acceleation is : %f" % response.acceleration)
            self.get_logger().info("The Steering angle is %f" % response.steering_angle)

            self.current_state = self.dummy_car_model(
                response.acceleration, response.steering_angle)
                
            self.request.initial_state_x = float(self.current_state[0])
            self.request.initial_state_y = float(self.current_state[1])
            self.request.initial_state_v = float(self.current_state[2])
            self.request.initial_state_theta = float(self.current_state[3])
            state_init = cd.DM([self.request.initial_state_x, 
                                self.request.initial_state_y, 
                                self.request.initial_state_v, 
                                self.request.initial_state_theta]
                                )
            print(state_init)
            
        except Exception as e:
            self.get_logger().error("The err is %r" % (e,))

    def dummy_car_model(self, acceleration, steering_angle):
        # Constructing a model which represents our car in carla
        temp_controls = cd.vertcat(acceleration, steering_angle)
        return cd.DM.full(self.current_state + (self.delta_T * self.non_linear_fun_generator(self.current_state, temp_controls)))


def main(args=None):
    rclpy.init(args=args)
    carlaClient = CarlaClient("carlaClient")
    rclpy.spin(carlaClient)
    carlaClient.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
