#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from abc import ABC, abstractmethod


class Solver(ABC):
    def __init__(self) -> None:
        super().__init__()
    
    @abstractmethod
    def set_options(self):
        pass

    @abstractmethod
    def set_constraints(self):
        pass

    @abstractmethod
    def set_objectives(self):
        pass
