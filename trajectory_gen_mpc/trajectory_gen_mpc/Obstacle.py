#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from abc import ABC, abstractmethod
from trajectory_gen_mpc.GlobalConstraints import ObstacleType

class Obstacle(ABC):
    def __init__(self) -> None:
        self.obstacle_type = ObstacleType
        super().__init__()
    
    @abstractmethod
    def set_position(self):
        pass
    @abstractmethod
    def get_obstacle_type(self):
        pass
