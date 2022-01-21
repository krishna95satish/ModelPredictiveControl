#!/usr/bin/env python3
import casadi as cd
from trajectory_gen_mpc.Obstacle import Obstacle


class DynamicObstacle(Obstacle):
    def __init__(self, obstacle_parameters) -> None:

        print('Dynamic Obstacle created')
        super().__init__()
        self.position_x = None
        self.position_y = None
        self.obstacle_diameter = None
        self.velocity_x = None
        self.velocity_y = None
        self.obstacle_constraints = []
        self.lower_bound_constraints = []
        self.upper_bound_constraints = []
        self.set_position(obstacle_parameters)

    def set_position(self, obstacle_parameters):

        self.position_x = obstacle_parameters.position_x
        self.position_y = obstacle_parameters.position_y
        self.obstacle_diameter = obstacle_parameters.obstacle_diameter
        self.velocity_x = obstacle_parameters.velocity_x
        self.velocity_y = obstacle_parameters.velocity_y
        self.delta_T = obstacle_parameters.Time_delta

    def set_collision_avoidance_constraints(self, prediction_horizon, ego_vehicle_diameter, state_X):

        self.lower_bound_constraints = cd.DM.zeros(((prediction_horizon+1), 1))
        self.upper_bound_constraints = cd.DM.zeros(((prediction_horizon+1), 1))
        # Formula : sqrt((x_robot - x_obstacle)^2 + (y_robot - y_obstacle)^2) >= robot_radius + obstacle_radius
        for idx in range(prediction_horizon+1):
            self.obstacle_constraints.append(-cd.sqrt((state_X[0, idx]-self.position_x)**2+(
                state_X[1, idx]-self.position_y)**2)+(ego_vehicle_diameter/2.+self.obstacle_diameter/2.))
            self.move_obstacle(self.position_x, self.position_y)

        self.lower_bound_constraints[0:(prediction_horizon+1)] = -cd.inf
        self.upper_bound_constraints[0:(prediction_horizon+1)] = 0

    def move_obstacle(self, pos_x, pos_y):
        # Constant velocity model : next_state = prev_state + delta_t * velocity
        self.position_x = self.position_x + (self.delta_T * self.velocity_x)
        self.position_y = self.position_y + (self.delta_T * self.velocity_y)


    def get_collision_avoidance_constraints(self):

        return self.obstacle_constraints, self.lower_bound_constraints, self.upper_bound_constraints
