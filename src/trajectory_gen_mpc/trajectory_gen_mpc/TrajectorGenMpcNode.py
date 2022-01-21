#!/usr/bin/env python3
import rclpy
import numpy as np
import casadi as cd
from rclpy.node import Node
from casadi import pi, inf
from custom_interfaces.msg import ControlAction, States
from trajectory_gen_mpc.NonHolonomic import NonHolonomic
from trajectory_gen_mpc.StaticObstacle import StaticObstacle
from trajectory_gen_mpc.DynamicObstacle import DynamicObstacle
from trajectory_gen_mpc.MultipleShooting import MultipleShooting
from trajectory_gen_mpc.GlobalConstraints import VehicleConstraints, SolverOptions, MpcTunableParameters, StaticObstacleParameters, DynamicObstacleParameters


class TrajectoryGenMpcNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.create_subscription(
            States, "Vehicle_states", self.solver_callback, 10)
        self.control_action_publisher = self.create_publisher(
            ControlAction, "Control_actions", 10)
        self.get_logger().info("TrajectoryNode has been initialized")
        self.control_actions = ControlAction()
        self.solver_setup()

    def solver_setup(self):
        vehicle_constrinats = VehicleConstraints(0.5,                               # wheel base
                                                 0.1,                               # acc_max            
                                                -0.1,                               # acc_min
                                                (pi/6),                             # steering_max
                                               -(pi/6),                             # steering_min
                                                 inf,                               # x_max
                                                -inf,                               # x_min
                                                 5.0,                               # y_max
                                                -5.0,                               # y_min
                                                 inf,                               # v_max
                                                -inf,                               # v_min
                                                 inf,                               # theta_max
                                                -inf,                               # theta_min
                                                 0.3                                # ego_vehicle_diameter
                                                 )               
                
        solver_options = SolverOptions(100,                                        # max_iter
                                          0,                                        # print_level
                                       1e-3,                                        # acceptable_tolerance
                                       1e-3,                                        # acceptable_objective_change_tolerance
                                          0                                         # print_time            
                                        )
        self.mpc_tunable_parameters = MpcTunableParameters(30 ,                     # prediction_horizon
                                                           40 ,                     # weight_on_x
                                                           20 ,                     # weight_on_y    
                                                            3 ,                     # weight_on_v    
                                                           18,                      # weight_on_theta
                                                           50 ,                     # weight_on_acceleration
                                                           215,                     # weight_on_steering_angle
                                                           395,                     # weight_on_change_in_steering_angle
                                                           0.3                      # delta_T
                                                            )
        static_obstacle1_parameter = StaticObstacleParameters(2.0,                  # position_x
                                                             0.8,                   # position_y
                                                              0.6                   # obstacle_diameter    
                                                              )
        dynamic_obstacle1_parameter = DynamicObstacleParameters(2.0,                # position_x
                                                               0.8,                 # position_y
                                                                0.6,                # obstacle_diameter
                                                                0.0,                # velocity_x
                                                                0.7,                # velocity_y
                              self.mpc_tunable_parameters.Time_delta                # delta_T
                                                                )

        obstacle_list = []
        static_obstacle1 = StaticObstacle(static_obstacle1_parameter)
        #dynamic_obstacle1 = DynamicObstacle(dynamic_obstacle1_parameter)
        obstacle_list.append(static_obstacle1)
        #obstacle_list.append(dynamic_obstacle1)

        self.car_altran = NonHolonomic(vehicle_constrinats)
        self.multiple_shooting_solver = MultipleShooting(self.car_altran, obstacle_list)
        self.multiple_shooting_solver.init_solver(solver_options, self.mpc_tunable_parameters)
        self.solver = self.multiple_shooting_solver.get_solver()

        self.u0 = cd.DM.zeros((self.car_altran.get_num_control(), self.mpc_tunable_parameters.prediction_horizon))  # initial control
        self.flag_first = True

    def solver_callback(self, states):
        state_init = cd.DM([states.initial_state_x,
                            states.initial_state_y,
                            states.initial_state_v,
                            states.initial_state_theta]
                           )
        state_target = cd.DM([states.final_state_x,
                              states.final_state_y,
                              states.final_state_v,
                              states.final_state_theta]
                             )
        print(state_init)
        if(self.flag_first):                     # avoid overwriting X0 variable on every callback
            self.X0 = cd.repmat(
                state_init, 1, self.mpc_tunable_parameters.prediction_horizon+1)
            self.flag_first = False

        lbg, ubg = self.multiple_shooting_solver.get_constraints()
        lbx, ubx = self.car_altran.get_constraints()
        args = {
            'lbg': lbg,  # constraints lower bound
            'ubg': ubg,  # constraints upper bound
            'lbx': lbx,
            'ubx': ubx
        }
        args['p'] = cd.vertcat(state_init,    # current state
                               state_target   # target state
                              )
        args['x0'] = cd.vertcat(cd.reshape(self.X0, self.car_altran.get_num_states() * (self.mpc_tunable_parameters.prediction_horizon+1), 1),
                                cd.reshape(self.u0, self.car_altran.get_num_control() * self.mpc_tunable_parameters.prediction_horizon, 1)
                               )
        sol = self.solver(
            x0  = args['x0'],
            lbx = args['lbx'],
            ubx = args['ubx'],
            lbg = args['lbg'],
            ubg = args['ubg'],
            p   = args['p']
            )
        u = cd.reshape(sol['x'][self.car_altran.get_num_states() * (self.mpc_tunable_parameters.prediction_horizon + 1):],
                      self.car_altran.get_num_control(), self.mpc_tunable_parameters.prediction_horizon)
        self.X0 = cd.reshape(sol['x'][: self.car_altran.get_num_states() * (self.mpc_tunable_parameters.prediction_horizon+1)],
                 self.car_altran.get_num_states(), self.mpc_tunable_parameters.prediction_horizon+1)

        print(float(u[0, 0]))
        print(float(u[1, 0]))

        self.control_actions.acceleration = float(u[0, 0])
        self.control_actions.steering_angle = float(u[1, 0])
        # Reinitializing X0 vector form the predicted states from the solver
        self.X0 = cd.horzcat(
            self.X0[:, 1:],
            cd.reshape(self.X0[:, -1], -1, 1)
        )
        # Refilling control action vector after extracting the first control action 
        self.u0 = cd.horzcat(u[:, 1:],
                  cd.reshape(u[:, -1], -1, 1))
        # Publishing first control actions
        self.control_action_publisher.publish(self.control_actions)


def main(args=None):
    rclpy.init(args=args)
    trajectory_node = TrajectoryGenMpcNode("Trajectory_node")
    rclpy.spin(trajectory_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
