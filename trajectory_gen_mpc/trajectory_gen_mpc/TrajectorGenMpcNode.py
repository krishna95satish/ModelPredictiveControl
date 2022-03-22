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
        self.solver_setup()

    def solver_setup(self):
        # TODO : Push the configuration literals into Yaml file
        vehicle_constrinats = VehicleConstraints(2.8,                              # wheel base (BMW GrandTourer)
                                                 1.0,                              # acc_max
                                                 -1.0,                             # acc_min
                                                 (pi/3),                           # steering_max 
                                                 -(pi/3),                          # steering_min
                                                 cd.inf,                              # x_max
                                                 -cd.inf,                             # x_min
                                                 cd.inf,                              # y_max
                                                 -cd.inf,                              # y_min
                                                 cd.inf,                              # v_max
                                                 -cd.inf,                             # v_min
                                                 cd.inf,                              # theta_max
                                                 -cd.inf,                             # theta_min
                                                 4.0                               # ego_vehicle_diameter
                                                 )

        solver_options = SolverOptions(2000,                                       # max_iter
                                       0,                                          # print_level
                                       1e-3,                                       # acceptable_tolerance
                                       1e-3,                                       # acceptable_objective_change_tolerance
                                       0                                           # print_time
                                       )
        self.mpc_tunable_parameters = MpcTunableParameters(50,                     # prediction_horizon
                                                           300,                    # weight_on_x
                                                           300,                    # weight_on_y
                                                           100,                    # weight_on_v
                                                           50,                    # weight_on_theta
                                                           1000,                    # weight_on_x(T)
                                                           1000,                    # weight_on_y(T)
                                                           0,                    # weight_on_v(T)
                                                           0,                    # weight_on_theta(T)
                                                           1000,                   # weight_on_acceleration
                                                           1000,                   # weight_on_steering_angle
                                                           1000,                   # weight_on_change_in_steering_angle(Negates oscillatory behaviour)
                                                           0.1                     # delta_T
                                                           )
        static_obstacle1_parameter = StaticObstacleParameters(38.5,             # position_x 
                                                              133.0,              # position_y
                                                              3.3                  # obstacle_diameter (Audi TT)
                                                              )
        static_obstacle2_parameter = StaticObstacleParameters(38.5,             # position_x 
                                                              125.0,              # position_y
                                                              3.3                  # obstacle_diameter (Audi TT)
                                                              )
        static_obstacle3_parameter = StaticObstacleParameters(34.5,             # position_x 
                                                              133.0,              # position_y
                                                              3.3                  # obstacle_diameter (Audi TT)
                                                              )       
        static_obstacle4_parameter = StaticObstacleParameters(34.5,             # position_x 
                                                              125.0,              # position_y
                                                              3.3                  # obstacle_diameter (Audi TT)
                                                              )                                                                                                                                 
        dynamic_obstacle1_parameter = DynamicObstacleParameters(
                                                                3.5,               # obstacle_diameter (Audi TT)
                                                                0.1                # delta_T (rate at which we are recieving info from CARLA)
                                                                )
        dynamic_obstacle2_parameter = DynamicObstacleParameters(
                                                                3.5,               # obstacle_diameter (Audi TT)
                                                                0.1                # delta_T (rate at which we are recieving info from CARLA)
                                                                )
        dynamic_obstacle3_parameter = DynamicObstacleParameters(
                                                                3.5,               # obstacle_diameter (Audi TT)
                                                                0.1                # delta_T (rate at which we are recieving info from CARLA)
                                                                )
        dynamic_obstacle4_parameter = DynamicObstacleParameters(
                                                                3.5,               # obstacle_diameter (Audi TT)
                                                                0.1                # delta_T (rate at which we are recieving info from CARLA)
                                                                )
        obstacle_list = []
        #static_obstacle1 = StaticObstacle(static_obstacle1_parameter)
        #static_obstacle2 = StaticObstacle(static_obstacle2_parameter)
        #static_obstacle3 = StaticObstacle(static_obstacle3_parameter)
        #static_obstacle4 = StaticObstacle(static_obstacle4_parameter)
        dynamic_obstacle1 = DynamicObstacle(dynamic_obstacle1_parameter)
        dynamic_obstacle2 = DynamicObstacle(dynamic_obstacle2_parameter)
        dynamic_obstacle3 = DynamicObstacle(dynamic_obstacle3_parameter)
        dynamic_obstacle4 = DynamicObstacle(dynamic_obstacle4_parameter)
        obstacle_list.append(dynamic_obstacle1)
        obstacle_list.append(dynamic_obstacle2)
        obstacle_list.append(dynamic_obstacle3)
        obstacle_list.append(dynamic_obstacle4)

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
        object_1_position = cd.DM([states.obstacle_1_state_x,
                                states.obstacle_1_state_y,
                                states.obstacle_1_velocity_x,
                                states.obstacle_1_velocity_y]
                                )
        object_2_position = cd.DM([states.obstacle_2_state_x,
                                states.obstacle_2_state_y,
                                states.obstacle_2_velocity_x,
                                states.obstacle_2_velocity_y]
                                )
        object_3_position = cd.DM([states.obstacle_3_state_x,
                                states.obstacle_3_state_y,
                                states.obstacle_3_velocity_x,
                                states.obstacle_3_velocity_y]
                                )
        object_4_position = cd.DM([states.obstacle_4_state_x,
                                states.obstacle_4_state_y,
                                states.obstacle_4_velocity_x,
                                states.obstacle_4_velocity_y]
                                )

        self.updated_vehicle_constraints.map_x_max = states.map_x_max
        self.updated_vehicle_constraints.map_x_min = states.map_x_min
        self.updated_vehicle_constraints.map_y_max = states.map_y_max
        self.updated_vehicle_constraints.map_y_min = states.map_y_min
        self.updated_vehicle_constraints.throttle_max = states.throttle_max
        self.updated_vehicle_constraints.throttle_min = states.throttle_min

        print(state_init)
        # avoid overwriting predicted_states variable on every callback
        if(self.flag_first):
            self.predicted_states = cd.repmat(
                state_init, 1, self.mpc_tunable_parameters.prediction_horizon+1)
            self.flag_first = False
        lbg, ubg = self.multiple_shooting_solver.get_constraints()
        if not (states.map_x_max == None):
            self.car_altran.update_vehicle_constraints(
                self.updated_vehicle_constraints)       # Handling dynamic map boundaries
        lbx, ubx = self.car_altran.get_constraints()
        args = {
            'lbg': lbg,  # constraints lower bound
            'ubg': ubg,  # constraints upper bound
            'lbx': lbx,
            'ubx': ubx
        }
        args['p'] = cd.vertcat(state_init,        # current state
                               state_target,      # target state
                               object_1_position,  # obstacle 1 position
                               object_2_position,  # obstacle 2 position
                               object_3_position,  # obstacle 3 position
                               object_4_position  # obstacle 4 position                                                                                             
                               )
        args['x0'] = cd.vertcat(cd.reshape(self.predicted_states, self.car_altran.get_num_states() * (self.mpc_tunable_parameters.prediction_horizon+1), 1),
                                cd.reshape(self.u0, self.car_altran.get_num_control(
                                ) * self.mpc_tunable_parameters.prediction_horizon, 1)
                                )
        sol = self.solver(
            x0=args['x0'],
            lbx=args['lbx'],
            ubx=args['ubx'],
            lbg=args['lbg'],
            ubg=args['ubg'],
            p=args['p']
        )
        control_actions = cd.reshape(sol['x'][self.car_altran.get_num_states() * (self.mpc_tunable_parameters.prediction_horizon + 1):],
                                     self.car_altran.get_num_control(), self.mpc_tunable_parameters.prediction_horizon)
        self.predicted_states = cd.reshape(sol['x'][: self.car_altran.get_num_states() * (self.mpc_tunable_parameters.prediction_horizon+1)],
                                           self.car_altran.get_num_states(), self.mpc_tunable_parameters.prediction_horizon+1)

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
