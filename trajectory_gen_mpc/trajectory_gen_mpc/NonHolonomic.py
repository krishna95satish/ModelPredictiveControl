#!/usr/bin/env python3
import casadi as cd
import numpy as np
from casadi import sin, cos, tan, pi
from numpy.core.fromnumeric import shape
from trajectory_gen_mpc.Vehicle import Vehicle


class NonHolonomic(Vehicle):
    def __init__(self, constraints) -> None:
        super().__init__()

        print("NonHolonomic car created")
        self.constraints = constraints
        self.lbx = []
        self.ubx = []
        self.states = []
        self.n_states = 0
        self.n_controls = 0
        self.controls = []
        self.prediction_horizon = 0
        self.non_linear_equ_generator = None
        # for collision avoidance calculation
        self.circular_approx_diameter = constraints.ego_vehicle_diameter

    def create_model(self):
        # This funciton will take the constraints and generate the bicycle model,
        # it returns a casADi function object which outputs the model with states and controls as inputs
        # defining states
        x = cd.SX.sym('x')
        y = cd.SX.sym('y')
        v = cd.SX.sym('v')
        theta = cd.SX.sym('theta')
        self.states = cd.vertcat(x,
                                 y,
                                 v,
                                 theta
                                 )
        self.n_states = self.states.numel()
        # defining controls
        a = cd.SX.sym('a')
        delta = cd.SX.sym('delta')
        self.controls = cd.vertcat(a,
                                   delta
                                   )
        self.n_controls = self.controls.numel()
        # define motion model equations
        self.non_linear_term = cd.vertcat(
            v*cos(theta),
            v*sin(theta),
            a,
            (v*(tan(delta)/self.constraints.wheel_base)) +
            (v*delta*tan(delta) ** 2) + delta/self.constraints.wheel_base
        )

    def get_model(self):
        # casADi function to create the motion model as x_dot = x + delta_T * non_linear_term
        self.non_linear_equ_generator = cd.Function(
            'f', [self.states, self.controls], [self.non_linear_term])
        return self.non_linear_equ_generator

    def set_vehicle_constraints(self, prediction_horizon):
        #################### State constraints ####################################
        self.lbx = cd.DM.zeros(
            (self.n_states*(prediction_horizon+1) + self.n_controls*prediction_horizon, 1))
        self.ubx = cd.DM.zeros(
            (self.n_states*(prediction_horizon+1) + self.n_controls*prediction_horizon, 1))
        self.prediction_horizon = prediction_horizon

        self.ubx[0: self.n_states*(prediction_horizon+1): self.n_states] = self.constraints.state1_max
        self.ubx[1: self.n_states*(prediction_horizon+1): self.n_states] = self.constraints.state2_max
        self.ubx[2: self.n_states*(prediction_horizon+1): self.n_states] = self.constraints.state3_max
        self.ubx[3: self.n_states*(prediction_horizon+1): self.n_states] = self.constraints.state4_max

        self.lbx[0: self.n_states*(prediction_horizon+1): self.n_states] = self.constraints.state1_min
        self.lbx[1: self.n_states*(prediction_horizon+1): self.n_states] = self.constraints.state2_min
        self.lbx[2: self.n_states*(prediction_horizon+1): self.n_states] = self.constraints.state3_min
        self.lbx[3: self.n_states*(prediction_horizon+1): self.n_states] = self.constraints.state4_min
        #################### Input constraints ####################################
        self.ubx[self.n_states*(prediction_horizon+1): self.n_states*(prediction_horizon+1)+(
            2*(prediction_horizon)): self.n_controls] = self.constraints.a_max
        self.ubx[self.n_states*(prediction_horizon+1)+1: self.n_states*(prediction_horizon+1)+(
            2*(prediction_horizon)): self.n_controls] = self.constraints.delta_max

        self.lbx[self.n_states*(prediction_horizon+1): self.n_states*(prediction_horizon+1)+(
            2*(prediction_horizon)): self.n_controls] = self.constraints.a_min
        self.lbx[self.n_states*(prediction_horizon+1)+1: self.n_states*(prediction_horizon+1)+(
            2*(prediction_horizon)): self.n_controls] = self.constraints.delta_min

    def update_vehicle_constraints(self, constraints):
        #################### State constraints ####################################
        self.ubx[0: self.n_states*(self.prediction_horizon+1): self.n_states] = constraints.map_x_max
        self.ubx[1: self.n_states*(self.prediction_horizon+1): self.n_states] = constraints.map_y_max

        self.lbx[0: self.n_states*(self.prediction_horizon+1): self.n_states] = constraints.map_x_min
        self.lbx[1: self.n_states*(self.prediction_horizon+1): self.n_states] = constraints.map_y_min

        #################### Input constraints ####################################
        self.ubx[self.n_states*(self.prediction_horizon+1): self.n_states*(self.prediction_horizon+1)+(
            2*(self.prediction_horizon)): self.n_controls] = constraints.throttle_max

        self.lbx[self.n_states*(self.prediction_horizon+1): self.n_states*(self.prediction_horizon+1)+(
            2*(self.prediction_horizon)): self.n_controls] = constraints.throttle_min


    def apply_first_control_action(self, state, control_action, delta_T, t0):

        non_linear_term = self.non_linear_equ_generator(
            state, control_action[:, 0])
        next_state = cd.DM.full(state + (delta_T * non_linear_term))
        t0 = t0 + delta_T
        updated_control_actions = cd.horzcat(control_action[:, 1:],
                                             cd.reshape(control_action[:, -1], -1, 1))

        return t0, next_state, updated_control_actions

    def get_states(self):
        return self.states

    def get_vehicle_diameter(self):
        return self.circular_approx_diameter

    def get_controls(self):
        return self.controls

    def get_num_states(self):
        return self.n_states

    def get_num_control(self):
        return self.n_controls

    def get_constraints(self):
        return self.lbx, self.ubx
