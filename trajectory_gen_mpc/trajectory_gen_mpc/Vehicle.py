#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from abc import ABC, abstractmethod


class Vehicle(ABC):
    def __init__(self) -> None:
        super().__init__()
        
    @abstractmethod
    def create_model(self):
        pass

    @abstractmethod
    def set_vehicle_constraints(self):
        pass
