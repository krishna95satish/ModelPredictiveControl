#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import Trajectory
from casadi import pi, inf
from time import time
import numpy as np
import casadi as cd
from trajectory_gen_mpc.NonHolonomic import NonHolonomic
from trajectory_gen_mpc.MultipleShooting import MultipleShooting
from trajectory_gen_mpc.StaticObstacle import StaticObstacle
from trajectory_gen_mpc.DynamicObstacle import DynamicObstacle
from trajectory_gen_mpc.GlobalConstraints import VehicleConstraints, SolverOptions, MpcTunableParameters, StaticObstacleParameters, DynamicObstacleParameters



class TrajectoryGenerator(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        vehicle_constrinats = VehicleConstraints(
            0.5, 0.1, -0.1, (pi/6), -(pi/6), inf, -inf, 5.0, -5.0, inf, -inf, inf, -inf, 0.3)
        solver_options = SolverOptions(1000, 0, 1e-3, 1e-3, 0)
        self.mpc_tunable_parameters = MpcTunableParameters(
            30, 50, 50, 4, 0.8, 50, 215, 295, 0.3)
        static_obstacle1_parameter = StaticObstacleParameters(3.0, -2.0, 0.6)
        dynamic_obstacle1_parameter = DynamicObstacleParameters(3.0, -2.0, 0.6, 0.3, 0.0, self.mpc_tunable_parameters.Time_delta)
        obstacle_list = []
        static_obstacle1 = StaticObstacle(static_obstacle1_parameter)
        #static_obstacle2 = StaticObstacle(static_obstacle1_parameter)
        dynamic_obstacle1 = DynamicObstacle(dynamic_obstacle1_parameter)
        obstacle_list.append(static_obstacle1)
        obstacle_list.append(dynamic_obstacle1)
        # obstacle_list.append(static_obstacle2)
        self.car_altran = NonHolonomic(vehicle_constrinats)

        self.multiple_shooting_solver = MultipleShooting(self.car_altran, obstacle_list)
        self.multiple_shooting_solver.init_solver(
            solver_options, self.mpc_tunable_parameters)
        self.solver = self.multiple_shooting_solver.get_solver()
        self.u0 = cd.DM.zeros((self.car_altran.get_num_control(),
                    self.mpc_tunable_parameters.prediction_horizon))  # initial control
        
        self.server = self.create_service(Trajectory, "TrajectoryServer", self.trajectory_Cb)
        self.get_logger().info("Trajectory Node has been created")
        self.flag_first = True

    def trajectory_Cb(self, Request, Response):
        state_init = cd.DM([Request.initial_state_x, 
                            Request.initial_state_y, 
                            Request.initial_state_v, 
                            Request.initial_state_theta]
                            )
        state_target = cd.DM([Request.final_state_x, 
                              Request.final_state_y, 
                              Request.final_state_v,
                              Request.final_state_theta]
                              )
        print(state_init)
        if(self.flag_first):
            self.X0 = cd.repmat(state_init, 1, self.mpc_tunable_parameters.prediction_horizon+1)
            self.flag_first = False
        
        lbg, ubg = self.multiple_shooting_solver.get_constraints()
        lbx, ubx = self.car_altran.get_constraints()

        args = {
            'lbg': lbg,  # constraints lower bound
            'ubg': ubg,  # constraints upper bound
            'lbx': lbx,
            'ubx': ubx
        }
        args['p'] = cd.vertcat(
            state_init,    # current state
            state_target   # target state
        )
        args['x0'] = cd.vertcat(
            cd.reshape(self.X0, self.car_altran.get_num_states() *
                       (self.mpc_tunable_parameters.prediction_horizon+1), 1),
            cd.reshape(self.u0, self.car_altran.get_num_control()
                       * self.mpc_tunable_parameters.prediction_horizon, 1)
        )
        sol = self.solver(
            x0=args['x0'],
            lbx=args['lbx'],
            ubx=args['ubx'],
            lbg=args['lbg'],
            ubg=args['ubg'],
            p=args['p']
        )
        u = cd.reshape(sol['x'][self.car_altran.get_num_states() *
                       (self.mpc_tunable_parameters.prediction_horizon + 1):], self.car_altran.get_num_control(), self.mpc_tunable_parameters.prediction_horizon)
        self.X0 = cd.reshape(sol['x'][: self.car_altran.get_num_states()
                        * (self.mpc_tunable_parameters.prediction_horizon+1)], self.car_altran.get_num_states(), self.mpc_tunable_parameters.prediction_horizon+1)
        
        Response.acceleration = float(u[0, 0])
        Response.steering_angle = float(u[1, 0])
        self.X0 = cd.horzcat(
            self.X0[:, 1:],
            # reinitializing with the predicted state form solver
            cd.reshape(self.X0[:, -1], -1, 1)
        )
        self.u0 = cd.horzcat(u[:, 1:], cd.reshape(u[:, -1], -1, 1))
        return Response



def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator("TrajectoryGenMPC")
    rclpy.spin(node)
    rclpy.shutdown() 



if __name__ == "__main__":
    main()