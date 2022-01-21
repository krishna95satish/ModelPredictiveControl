#!/usr/bin/env python3
from __future__ import print_function
from re import S
import casadi as cd
from trajectory_gen_mpc.Solver import Solver


class MultipleShooting(Solver):
    def __init__(self, car_object, obstacle_list=None) -> None:

        super().__init__()
        self.car = car_object
        self.num_of_states = 0
        self.num_of_controls = 0
        self.obstacle_states = 4
        self.num_of_slack_var = 2
        self.obstacle_list = obstacle_list
        self.objective = 0
        self.prediction_horizon = 0
        self.constraints_vector = []        # equality and other constrains vector
        self.solver_options = None
        self.X = []
        self.P = []
        self.Q = []
        self.U = []
        self.R = []
        self.Z = []
        self.T = []
        self.S = []
        self.ST = []
        self.solver = None
        self.solver_options = None
        self.optimization_variables = []

    def set_optimization_variables(self):

        self.optimization_variables = cd.vertcat(
            cd.reshape(self.X, -1, 1),      # States          [x, y, v, theta]
            # Control actione [a, steering angle]
            cd.reshape(self.U, -1, 1),
            cd.reshape(self.S, -1, 1)      # Slack variable
        )

    def init_solver(self, options, tunable_parameters):
        # This methos initializes and sets up the required components for a Solver

        # creates the Kinematic motion model in Vehicle class
        self.car.create_model()
        self.num_of_states = self.car.get_num_states()
        self.num_of_controls = self.car.get_num_control()
        self.lamda_weight = tunable_parameters.lambda_weight
        self.prediction_horizon = tunable_parameters.prediction_horizon
        self.X = cd.SX.sym('X', self.num_of_states,
                           (tunable_parameters.prediction_horizon+1))
        # obstacle states x, y , v_x and v_y
        self.P = cd.SX.sym('P', self.num_of_states +
                           self.num_of_states + self.obstacle_states)
        self.U = cd.SX.sym('U', self.num_of_controls,
                           tunable_parameters.prediction_horizon)
        self.Q = cd.diagcat(tunable_parameters.weight_state_1, tunable_parameters.weight_state_2,
                            tunable_parameters.weight_state_3, tunable_parameters.weight_state_4)      # weight matrix on states (x, y, v, theta)
        self.TR = cd.diagcat(tunable_parameters.weight_state_5, tunable_parameters.weight_state_6,
                            tunable_parameters.weight_state_7, tunable_parameters.weight_state_8)      # weight matrix on states (x, y, v, theta)
        self.R = cd.diagcat(
            tunable_parameters.weight_control_1, tunable_parameters.weight_control_2)                  # weight matrix on constrol inputs (a, delta)
        self.S = cd.SX.sym('S', self.num_of_slack_var,
                           tunable_parameters.prediction_horizon+1)
        self.ST = cd.diagcat(tunable_parameters.weight_slackvar_x,
                             tunable_parameters.weight_slackvar_y)
        self.slack_lower_bound_constraints = cd.DM.zeros((self.num_of_slack_var*(self.prediction_horizon+1), 1))
        self.slack_upper_bound_constraints = cd.DM.zeros((self.num_of_slack_var*(self.prediction_horizon+1), 1))
        # change in steering angle
        self.Z = tunable_parameters.weight_delta
        # Time T
        self.T = tunable_parameters.Time_delta
        self.set_options(options)
        self.car.set_vehicle_constraints(tunable_parameters.prediction_horizon)
        self.set_constraints()
        self.set_optimization_variables()
        self.set_objectives()
        self.setup_obstacles(tunable_parameters.prediction_horizon,
                             self.car.get_vehicle_diameter(), self.X, self.P[8:])
        self.set_slack_variable_constraints()
        self.formulate_NLP()
        self.create_solver()

    def set_options(self, options):
        # Setting up the IPOPT solver

        self.solver_options = {
            'ipopt': {
                'max_iter': options.max_iter,
                'print_level': options.print_level,
                'acceptable_tol': options.acceptable_tolerance,
                'acceptable_obj_change_tol': options.acceptable_objective_change_tolerance
            },
            'print_time': options.print_time
        }

    def setup_obstacles(self, prediction_horizon, ego_vehicle_diameter, state_X, P_vector):
        # This methods initiates the obstacles if there are any provided in obstacle list in constructor

        if (self.obstacle_list != None):
            for idx in range(len(self.obstacle_list)):
                self.obstacle_list[idx].set_collision_avoidance_constraints(                                         # Similar to Dynamic
                    prediction_horizon, ego_vehicle_diameter, state_X, P_vector)
                obstacle_constraints, lower_bounds, upper_bounds = self.obstacle_list[idx].get_collision_avoidance_constraints(
                )
                for constraints in obstacle_constraints:
                    self.constraints_vector = cd.vertcat(
                        self.constraints_vector, constraints)
                self.lb_constraints_vector = cd.vertcat(
                    self.lb_constraints_vector, lower_bounds)
                self.ub_constraints_vector = cd.vertcat(
                    self.ub_constraints_vector, upper_bounds)
        else:
            # DO Nothing since there are no obstacles
            pass

    def set_constraints(self):
        # Constaints for Multiple Shooting technique and Obstacle avaidance

        self.lb_constraints_vector = cd.DM.zeros(
            (self.num_of_states*(self.prediction_horizon+1), 1))
        self.ub_constraints_vector = cd.DM.zeros(
            (self.num_of_states*(self.prediction_horizon+1), 1))
        self.set_terminal_constraints()

    def set_terminal_constraints(self):
        temp_lb_terminal_constraints = cd.DM.zeros(self.num_of_states, 1)
        temp_lb_terminal_constraints[0:self.num_of_states] = 0
        temp_ub_terminal_constraints = cd.DM.zeros(self.num_of_states, 1)
        temp_ub_terminal_constraints[0:self.num_of_states] = cd.inf
        self.lb_constraints_vector = cd.vertcat(self.lb_constraints_vector, temp_lb_terminal_constraints)
        self.ub_constraints_vector = cd.vertcat(self.ub_constraints_vector, temp_ub_terminal_constraints)
        

    def get_constraints(self):
        return self.lb_constraints_vector, self.ub_constraints_vector

    def set_objectives(self):

        # initialization of steering for change in steering constraint
        prev_steering = 0
        # X : [x_1, x_2 ....x_N+1], P : [Initial and Fianl States]
        self.constraints_vector = self.X[:, 0] - self.P[:self.num_of_states]
        for idx in range(self.prediction_horizon):
            temp_state = self.X[:, idx]
            temp_control = self.U[:, idx]
            temp_slack = self.S[:, idx]
            # objective_fun = current_state - reference_state * weight_matrix + currect_input - ref_input * weight_matrix + Prev_steering - Current_steering * Weight
            self.objective += (temp_state-self.P[4:8]).T @ self.Q @ (temp_state - self.P[4:8]) + (
                temp_control).T @ self.R @ temp_control + temp_control[1]-prev_steering*self.Z*(temp_control[1]-prev_steering).T + self.lamda_weight * ((temp_slack).T @ self.ST @ temp_slack)
            prev_steering = temp_control[1]
            temp_next_state = self.X[:, idx+1]
            # Runge Kutta based descritization for smooth trajectory
            biycle_model = self.car.get_model()
            k1 = biycle_model(temp_state, temp_control)
            k2 = biycle_model(temp_state + self.T/2*k1, temp_control)
            k3 = biycle_model(temp_state + self.T/2*k2, temp_control)
            k4 = biycle_model(temp_state + self.T*k3, temp_control)
            st_next_motion_model = temp_state + \
                self.T/6*(k1 + 2*k2 + 2*k3 + k4)
            self.constraints_vector = cd.vertcat(
                self.constraints_vector, temp_next_state - st_next_motion_model)
        # Trerminal costs
        self.objective += (temp_state - self.P[4:8]).T @ self.TR @ (temp_state - self.P[4:8])
        self.constraints_vector = cd.vertcat(self.constraints_vector, temp_state - self.P[4:8])

    def set_slack_variable_constraints(self):
        for idx in range(self.prediction_horizon+1):
            self.constraints_vector = cd.vertcat(self.constraints_vector, self.X[0,idx] - self.S[0, idx])
            self.constraints_vector = cd.vertcat(self.constraints_vector, self.X[0,idx] + self.S[0, idx])
            self.constraints_vector = cd.vertcat(self.constraints_vector, self.X[1,idx] - self.S[1, idx])
            self.constraints_vector = cd.vertcat(self.constraints_vector, self.X[1,idx] + self.S[1, idx])
        for idx in range(self.num_of_slack_var):          # 
            self.set_slack_variable_bounds()
            self.update_constraint_bounds()
    
    def set_slack_variable_bounds(self):

        self.slack_lower_bound_constraints[0 : self.num_of_slack_var*(self.prediction_horizon+1) : self.num_of_slack_var] = -cd.inf
        self.slack_lower_bound_constraints[1 : self.num_of_slack_var*(self.prediction_horizon+1) : self.num_of_slack_var] = -cd.inf # actual map min bound goes here
        self.slack_upper_bound_constraints[0 : self.num_of_slack_var*(self.prediction_horizon+1) : self.num_of_slack_var] = cd.inf  # actual map max bound goes here
        self.slack_upper_bound_constraints[1 : self.num_of_slack_var*(self.prediction_horizon+1) : self.num_of_slack_var] = cd.inf

    def update_constraint_bounds(self):

        self.lb_constraints_vector = cd.vertcat(self.lb_constraints_vector, self.slack_lower_bound_constraints)
        self.ub_constraints_vector = cd.vertcat(self.ub_constraints_vector, self.slack_upper_bound_constraints) 
    
    def update_soft_constraints(self, first_slack_upper_bound, first_slack_lower_bound, second_slack_upper_bound, second_slack_lower_bound):
        if ((first_slack_upper_bound < first_slack_lower_bound) or (second_slack_upper_bound < second_slack_lower_bound)):
            print("Error: Lower bound values cannot be higher than upper bound values, Exiting!")
            raise ValueError('Lower bound values cannot be higher than upper bound values')
        temp_end_idx = self.lb_constraints_vector.size()[0]
        temp_start_idx = temp_end_idx - (2*self.num_of_slack_var * (self.prediction_horizon+1))
        temp_upper_bounds = [first_slack_upper_bound, second_slack_upper_bound]
        temp_lower_bounds = [first_slack_lower_bound, second_slack_lower_bound]
        for idx in range(self.num_of_slack_var):
            self.ub_constraints_vector[temp_start_idx  : temp_start_idx + 2*(self.prediction_horizon+1) : self.num_of_slack_var] = temp_upper_bounds[idx]
            self.lb_constraints_vector[temp_start_idx + 1 : temp_start_idx + 2*(self.prediction_horizon+1) : self.num_of_slack_var] = temp_lower_bounds[idx]
            temp_start_idx = temp_start_idx + 2*(self.prediction_horizon+1)
    
    def update_soft_constraints_free_space(self, ego_current_state, freeSpaceObject):
        ################### EGO DATA ##################################
        current_ego_position = ego_current_state[0]
        current_ego_velocity = ego_current_state[2]
        current_ego_orientation = ego_current_state[3]
        ################## EGO DATA ENDS ##############################
        temp_end_idx = self.lb_constraints_vector.size()[0]
        temp_start_idx = temp_end_idx - (2*self.num_of_slack_var * (self.prediction_horizon+1))
        for idx in range(self.num_of_slack_var):
            if idx <= 0:
                temp_start_idx = temp_start_idx + 2*(self.prediction_horizon+1)
                continue
            for idy in range(temp_start_idx, temp_start_idx + 2*(self.prediction_horizon+1), self.num_of_slack_var):
                # from carla we know that right hand turn results in lesser y axis values and left turn results in greater y axis values
                # hence, we consider lowebounds for right turn and upperbounds for left turn
                current_right_lane_boundary, current_left_lane_boundary = freeSpaceObject.extract_boundaies(current_ego_position)                           # ego_cuurent_state[0] -> where is the ego in x axis on the map
                self.ub_constraints_vector[idy] = current_left_lane_boundary
                self.lb_constraints_vector[idy+1] = current_right_lane_boundary
                current_ego_position = current_ego_position + (0.1 * current_ego_velocity*cd.cos(current_ego_orientation)) # Next_state = current_Sate + (delta_T * V*cos(theta)) 
            temp_start_idx = temp_start_idx + 2*(self.prediction_horizon+1)

    def formulate_NLP(self):

        self.nlp_prob = {'f': self.objective,                      # Objective
                         'x': self.optimization_variables,         # contains both X and U
                         'p': self.P,                              # vector with initial and reference points
                         # constraints for collision avoidance and multiple shooting
                         'g': self.constraints_vector
                         }

    def create_solver(self):
        self.solver = cd.nlpsol(
            'solver', 'ipopt', self.nlp_prob, self.solver_options)

    def get_solver(self):
        return self.solver
