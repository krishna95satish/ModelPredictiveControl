#!/usr/bin/env python3
"""Trajectory_Gen_MPC_Node responsible for creating object of MPC class and initializing all the required constraints and obstacles
   Subscribes : Vehicle states and generates and publishes control actions
   Calculates : Control actions and predicted states
   Publishes  : First control action 
"""
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
from trajectory_gen_mpc.GlobalConstraints import VehicleConstraints, \
    SolverOptions, MpcTunableParameters, StaticObstacleParameters, \
    DynamicObstacleParameters, UpdatedVehicleConstraints
from trajectory_gen_mpc.DummyFreeSpaceServer import DummyFreeSpaceServer


class TrajectoryGenMpcNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.create_subscription(
            States, "Vehicle_states", self.solver_callback, 10)
        self.control_action_publisher = self.create_publisher(
            ControlAction, "Control_actions", 10)
        self.get_logger().info("TrajectoryNode has been initialized")
        self.control_actions = ControlAction()
        self.updated_vehicle_constraints = UpdatedVehicleConstraints()
        self.freeSpaceServer = DummyFreeSpaceServer()
        self.solver_setup()

    def solver_setup(self):
        # TODO : Push the configuration literals into Yaml file
        vehicle_constrinats = VehicleConstraints(2.78,                              # wheel base (BMW GrandTourer)
                                                 0.5,                              # acc_max
                                                 -0.5,                             # acc_min
                                                 # steering_max
                                                 (pi/6),
                                                 # steering_min
                                                 -(pi/6),
                                                 inf,                              # x_max
                                                 -inf,                             # x_min
                                                 inf,                              # y_max
                                                 -inf,                              # y_min
                                                 inf,                              # v_max
                                                 -inf,                             # v_min
                                                 inf,                              # theta_max
                                                 -inf,                             # theta_min
                                                 4.5                               # ego_vehicle_diameter
                                                 )

        solver_options = SolverOptions(2000,                                       # max_iter
                                       0,                                          # print_level
                                       1e-10,                                       # acceptable_tolerance
                                       1e-10,                                       # acceptable_objective_change_tolerance
                                       0                                           # print_time
                                       )
        self.mpc_tunable_parameters = MpcTunableParameters(40,                     # prediction_horizon
                                                           500,                    # weight_on_x
                                                           300,                    # weight_on_y
                                                           100,                    # weight_on_v
                                                           900,                    # weight_on_theta
                                                           500,                   # terminal weight on x
                                                           100,                   # terminal weight on y
                                                           10,                   # terminal weight on v
                                                           100,                   # terminal weight on theta
                                                           1000,                   # weight_on_acceleration
                                                           1000,                   # weight_on_steering_angle
                                                           1,                      # weight_on_slack_variable_x
                                                           1,                      # weight_on_slack_variable_y
                                                           18,                # Weight on Lambda for Slack variables
                                                           # weight_on_change_in_steering_angle(Negates oscillatory behaviour)
                                                           10000,
                                                           0.1                     # delta_T
                                                           )

        static_obstacle1_parameter = StaticObstacleParameters(-1205.0,             # position_x
                                                              -910.4,              # position_y
                                                              # obstacle_diameter (Audi TT)
                                                              4.5
                                                              )
        dynamic_obstacle1_parameter = DynamicObstacleParameters(-2.0,              # position_x
                                                                0.8,               # position_y
                                                                # obstacle_diameter (Audi TT)
                                                                4.5,
                                                                0.0,               # velocity_x
                                                                0.0,               # velocity_y
                                                                # delta_T (rate at which we are recieving info from CARLA)
                                                                0.1
                                                                )

        obstacle_list = []
        #static_obstacle1 = StaticObstacle(static_obstacle1_parameter)
        dynamic_obstacle1 = DynamicObstacle(dynamic_obstacle1_parameter)
        # obstacle_list.append(static_obstacle1)
        obstacle_list.append(dynamic_obstacle1)

        self.car_altran = NonHolonomic(vehicle_constrinats)
        self.multiple_shooting_solver = MultipleShooting(
            self.car_altran, obstacle_list)
        self.multiple_shooting_solver.init_solver(
            solver_options, self.mpc_tunable_parameters)
        self.solver = self.multiple_shooting_solver.get_solver()
        self.u0 = cd.DM.zeros((self.car_altran.get_num_control(
        ), self.mpc_tunable_parameters.prediction_horizon))  # initial control
        # initial slack values
        self.sl0 = cd.DM.zeros(
            (2, self.mpc_tunable_parameters.prediction_horizon+1))
        self.flag_first = True
        # initialize freeSpaceServer
        self.freeSpaceServer.create_dummy_map()
        self.freeSpaceServer.interpolate_map_boundaries()

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
        obstacle_position = cd.DM([states.obstacle_state_x,
                                states.obstacle_state_y,
                                states.obstacle_velocity_x,
                                states.obstacle_velocity_y]
                                )
        #self.updated_vehicle_constraints.map_x_max = states.map_x_max
        #self.updated_vehicle_constraints.map_x_min = states.map_x_min
        #self.updated_vehicle_constraints.map_y_max = states.map_y_max
        #self.updated_vehicle_constraints.map_y_min = states.map_y_min
        #self.updated_vehicle_constraints.throttle_max = states.throttle_max
        #self.updated_vehicle_constraints.throttle_min = states.throttle_min

        print(state_init)
        # avoid overwriting predicted_states variable on every callback
        if(self.flag_first):
            self.predicted_states = cd.repmat(
                state_init, 1, self.mpc_tunable_parameters.prediction_horizon+1)
            self.flag_first = False
        ############################ SOFT CONSTRAINTS BEGIN ######################################
        #self.multiple_shooting_solver.update_soft_constraints(cd.inf, -cd.inf, cd.inf, -cd.inf)   # upper, lower , upper, lower
        ############################ SOFT CONSTRAINTS END ######################################
        #lbg, ubg = self.multiple_shooting_solver.get_constraints()
        #if not (states.map_x_max == None):
        #    self.car_altran.update_vehicle_constraints(self.updated_vehicle_constraints)       # Handling dynamic map boundaries
        self.car_altran.update_vehicle_constraints_free_space(state_init, self.freeSpaceServer)
        #self.multiple_shooting_solver.update_soft_constraints_free_space(state_init, self.freeSpaceServer)
        lbg, ubg = self.multiple_shooting_solver.get_constraints()
        lbx, ubx = self.car_altran.get_constraints()
        ############# temp workaround for slack variable testing ######################
        temp_lbx_slack = cd.DM.zeros(
            (2*(self.mpc_tunable_parameters.prediction_horizon+1), 1))
        temp_ubx_slack = cd.DM.zeros(
            (2*(self.mpc_tunable_parameters.prediction_horizon+1), 1))
        temp_lbx_slack[0: 2 *
                       (self.mpc_tunable_parameters.prediction_horizon+1): 2] = 0
        temp_ubx_slack[0: 2 *
                       (self.mpc_tunable_parameters.prediction_horizon+1): 2] = cd.inf
        temp_lbx_slack[1: 2 *
                       (self.mpc_tunable_parameters.prediction_horizon+1): 2] = 0
        temp_ubx_slack[1: 2 *
                       (self.mpc_tunable_parameters.prediction_horizon+1): 2] = cd.inf
        lbx = cd.vertcat(lbx, temp_lbx_slack)
        ubx = cd.vertcat(ubx, temp_ubx_slack)
        ############## temp workaround ends ###########################################
        args = {
            'lbg': lbg,  # constraints lower bound
            'ubg': ubg,  # constraints upper bound
            'lbx': lbx,
            'ubx': ubx
        }
        args['p'] = cd.vertcat(state_init,        # current state
                               state_target,      # target state
                               obstacle_position  # obstacle position
                               )
        args['x0'] = cd.vertcat(cd.reshape(self.predicted_states, self.car_altran.get_num_states() * (self.mpc_tunable_parameters.prediction_horizon+1), 1),
                                cd.reshape(self.u0, self.car_altran.get_num_control(
                                ) * self.mpc_tunable_parameters.prediction_horizon, 1),
                                cd.reshape(
                                    self.sl0, 2 * (self.mpc_tunable_parameters.prediction_horizon+1), 1)
                                )
        sol = self.solver(
            x0=args['x0'],
            lbx=args['lbx'],
            ubx=args['ubx'],
            lbg=args['lbg'],
            ubg=args['ubg'],
            p=args['p']
        )
        control_actions = cd.reshape(sol['x'][self.car_altran.get_num_states() * (self.mpc_tunable_parameters.prediction_horizon + 1) : self.car_altran.get_num_states() * (self.mpc_tunable_parameters.prediction_horizon + 1) + (self.car_altran.get_num_control() * self.mpc_tunable_parameters.prediction_horizon)],
                                     self.car_altran.get_num_control(), self.mpc_tunable_parameters.prediction_horizon)
        self.predicted_states = cd.reshape(sol['x'][: self.car_altran.get_num_states() * (self.mpc_tunable_parameters.prediction_horizon+1)],
                                           self.car_altran.get_num_states(), self.mpc_tunable_parameters.prediction_horizon+1)
        self.sl0 = cd.reshape(sol['x'][self.car_altran.get_num_states() * (self.mpc_tunable_parameters.prediction_horizon + 1) +
                              self.car_altran.get_num_control() * self.mpc_tunable_parameters.prediction_horizon:], 2, (self.mpc_tunable_parameters.prediction_horizon + 1))

        print(float(control_actions[0, 0]))
        print(float(control_actions[1, 0]))

        self.control_actions.acceleration = float(control_actions[0, 0])
        self.control_actions.steering_angle = float(control_actions[1, 0])
        # Reinitializing predicted_states vector form the the solver's predicted states
        self.predicted_states = cd.horzcat(
            self.predicted_states[:, 1:],
            cd.reshape(self.predicted_states[:, -1], -1, 1)
        )
        # Refilling control action vector after extracting the first control action
        self.u0 = cd.horzcat(control_actions[:, 1:],
                             cd.reshape(control_actions[:, -1], -1, 1))
        # Publishing first control actions
        self.control_action_publisher.publish(self.control_actions)


def main(args=None):
    rclpy.init(args=args)
    trajectory_node = TrajectoryGenMpcNode("Trajectory_Gen_MPC_Node")
    rclpy.spin(trajectory_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
