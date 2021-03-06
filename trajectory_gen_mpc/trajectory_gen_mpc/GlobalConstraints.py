#!/usr/bin/env python3
"""Global_Constraints : File with structures for all the constaints and constants
   Subscribes : NA
   Calculates : NA
   Publishes  : NA 
"""
from typing import NamedTuple
from enum import Enum


class VehicleConstraints(NamedTuple):
    # This class constains all the constraints for the Car's state and inputs namely
    # x, y, v, theta and acceleration and steering angleu
    wheel_base:           float
    a_max:                float
    a_min:                float
    delta_max:            float
    delta_min:            float
    state1_max:           float       # state x max
    state1_min:           float       # state x min
    state2_max:           float       # state y max
    state2_min:           float       # state y min
    state3_max:           float       # state v max
    state3_min:           float       # state v min
    state4_max:           float       # state theta max
    state4_min:           float       # state theta min
    ego_vehicle_diameter: float


class SolverOptions(NamedTuple):
    # This Class consists of all the options related to the Solver (IPOPT)
    max_iter: int
    print_level: int
    acceptable_tolerance: float
    acceptable_objective_change_tolerance: float
    print_time: int


class MpcTunableParameters(NamedTuple):
    # This class consists of the tunable parametes including weight matrix and the prediction horizon
    prediction_horizon: int
    weight_state_1:     float     # Penality on x
    weight_state_2:     float     # Penality on y
    weight_state_3:     float     # Penality on v
    weight_state_4:     float     # Penality on theta
    weight_state_5:     float     # Penality on x(Terminal)
    weight_state_6:     float     # Penality on y(Terminal)
    weight_state_7:     float     # Penality on v(Terminal)
    weight_state_8:     float     # Penality on theta(Terminal)
    weight_control_1:   float     # Penality on acceleration
    weight_control_2:   float     # Penality on steeringAngle
    weight_delta:       float     # Penality on change in steering angle
    Time_delta:         float     # delta T in seconds


class StaticObstacleParameters(NamedTuple):
    # This class constaints the ostacle positions
    position_x:         float
    position_y:         float
    obstacle_diameter:  float

class DynamicObstacleParameters(NamedTuple):
    # This class constaints the ostacle positions and velocities 
    obstacle_diameter:  float
    Time_delta:         float

class UpdatedVehicleConstraints:
    # This is a datastructure to handle changing map boundaries
    map_x_max    = None
    map_x_min    = None
    map_y_max    = None
    map_y_min    = None
    throttle_max = None
    throttle_min = None

class ObstacleType(Enum):
    """Enum class used to diffenciate static vs dynamic obstacles"""
    static_obstacle  = 1
    dynamic_obstacle = 2