#!/usr/bin/env python3
import casadi as cd
from trajectory_gen_mpc.Obstacle import Obstacle


class StaticObstacle(Obstacle):
    def __init__(self, obstacle_parameters) -> None:

        print('Static Obstacle created')
        super().__init__()
        self.position_x = None
        self.position_y = None
        self.obstacle_diameter = None
        self.obstacle_constraints = []
        self.lower_bound_constraints = []
        self.upper_bound_constraints = []
        self.set_position(obstacle_parameters)

    def set_position(self, obstacle_parameters):

        self.position_x = obstacle_parameters.position_x
        self.position_y = obstacle_parameters.position_y
        self.obstacle_diameter = obstacle_parameters.obstacle_diameter

    def set_collision_avoidance_constraints(self, prediction_horizon, ego_vehicle_diameter, state_X):

        self.lower_bound_constraints = cd.DM.zeros(((prediction_horizon+1), 1))
        self.upper_bound_constraints = cd.DM.zeros(((prediction_horizon+1), 1))
        # Formula : sqrt((x_robot - x_obstacle)^2 + (y_robot - y_obstacle)^2) >= robot_radius + obstacle_radius
        for idx in range(prediction_horizon+1):
            self.obstacle_constraints.append(-cd.sqrt((state_X[0, idx]-self.position_x)**2+(
                state_X[1, idx]-self.position_y)**2)+(ego_vehicle_diameter/2.+self.obstacle_diameter/2.))

        self.lower_bound_constraints[0:(prediction_horizon+1)] = -cd.inf
        self.upper_bound_constraints[0:(prediction_horizon+1)] = 0

    def get_collision_avoidance_constraints(self):

        return self.obstacle_constraints, self.lower_bound_constraints, self.upper_bound_constraints
