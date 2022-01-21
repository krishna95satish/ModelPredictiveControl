#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from abc import ABC, abstractmethod

class Obstacle(ABC):
    def __init__(self) -> None:
        super().__init__()
    
    @abstractmethod
    def set_position(self):
        pass
