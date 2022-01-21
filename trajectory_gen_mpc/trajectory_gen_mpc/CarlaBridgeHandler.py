#!/usr/bin/env python3
"""Carla_Bridge_Handler_Node responsible for communicating with CARLA and Trajectory_Gen_MPC_Node
   Subscribes : Ego_Vehicle_odometry, Velocity, Orientation
   Calculates : Normalizing control actions, Quaternion to Euler angle converstion
   Publishes  : Steering angle, Throttle  
"""

import rclpy
import numpy as np
import casadi as cd
from rclpy.node import Node
from nav_msgs.msg import Odometry
from custom_interfaces.msg import States, ControlAction
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus


class CarlaBridgeHandler(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info("Carla Handler Initiated")
        self.states = States()
        self.vehicle_controls = CarlaEgoVehicleControl()
        # Final State hard coded for now [ONLY FOR TESTING]
        self.states.final_state_x = -122.0
        # for intersection : x : -122.0, y : -110.0 v : 0.0 theta : -(cd.pi/2) [Right turn]
        self.states.final_state_y = -120.4
        # for intersection : x : -120.0, y : -80.0 v : 0.0 theta : (cd.pi/2) [Left turn]
        self.states.final_state_v = 0.0
        self.states.final_state_theta =  -(cd.pi/2)
        self.current_state = None
        self.target_state = None
        # Terminal constraits
        self.mpc_iter = 0
        self.error_tolerance = 1.0
        self.is_target_reached = False
        # Pub-Sub callbacks
        self.ego_pose_subscriber = self.create_subscription(
            Odometry, "/carla/vehicle_00/odometry", self.ego_pose_callback, 10)
        self.obstacle_pose_subscriber = self.create_subscription(
            Odometry, "/carla/vehicle_01/odometry", self.obstacle_pose_callback, 10)
        self.vehicle_control_publisher = self.create_publisher(
            CarlaEgoVehicleControl, '/carla/vehicle_00/vehicle_control_cmd', 10)
        self.ego_velocity_subscriber = self.create_subscription(CarlaEgoVehicleStatus, "/carla/vehicle_00/vehicle_status",
                                                                self.ego_vehicle_status_callback, 10)
        self.obstacle_velocity_subscriber = self.create_subscription(CarlaEgoVehicleStatus, "/carla/vehicle_01/vehicle_status",
                                                                     self.obstacle_vehicle_status_callback, 10)
        self.state_publisher = self.create_publisher(
            States, "Vehicle_states", 10)
        self.create_subscription(
            ControlAction, "Control_actions", self.control_action_callback, 10)

    def update_state_vector(self):
        self.current_state = cd.DM([self.states.initial_state_x,
                                    self.states.initial_state_y,
                                    self.states.initial_state_v,
                                    self.states.initial_state_theta]
                                   )
        self.target_state = cd.DM([self.states.final_state_x,
                                   self.states.final_state_y,
                                   self.states.final_state_v,
                                   self.states.final_state_theta]
                                  )

    def ego_pose_callback(self, ego_pose):
        try:
            # Check to stop running MPC after reaching the target
            if not(self.is_target_reached):
                self.states.initial_state_x = ego_pose.pose.pose.position.x
                self.states.initial_state_y = ego_pose.pose.pose.position.y
                self.states.initial_state_v = self.ego_velocity
                self.states.initial_state_theta = self.calculate_yaw_from_quaternion(
                    self.ego_orientaion_quaternion)
                self.update_state_vector()
                # Updating map boundaries on each step
                self.update_map_boundaries()
                self.state_publisher.publish(self.states)
        except AttributeError:
            print("Please rerun the node, HINT : Callback Sync Failure")

    def obstacle_pose_callback(self, obstacle_pose):
        try:
            if not(self.is_target_reached):
                self.states.obstacle_state_x = obstacle_pose.pose.pose.position.x
                self.states.obstacle_state_y = obstacle_pose.pose.pose.position.y
                self.obstacle_orientation = self.calculate_yaw_from_quaternion(
                    self.obstacle_orientaion_quaternion)
                self.states.obstacle_velocity_x = self.obstacle_velocity * \
                    np.cos(self.obstacle_orientation)  # V_x = V * cos(theta)
                self.states.obstacle_velocity_y = self.obstacle_velocity * \
                    np.sin(self.obstacle_orientation)  # V_y = V * sin(theta)
        except AttributeError:
            print("Please rerun the node, HINT : Callback Sync Failure")
        except KeyboardInterrupt:
            print("Interrupted")

    def update_map_boundaries(self):
        # Dummy funtion to emulate the "Free Space" requirements in AORTA
        if (self.states.initial_state_x < -130.0):
            self.states.map_x_max = cd.inf
            self.states.map_x_min = -cd.inf
            self.states.map_y_max = cd.inf  # -91.4
            self.states.map_y_min = -cd.inf  # -91.4
            self.states.throttle_max = 0.5
            self.states.throttle_min = -0.5
        elif (self.states.initial_state_x > -130.0):
            self.states.map_x_max = cd.inf
            self.states.map_x_min = -cd.inf
            self.states.map_y_max = cd.inf
            self.states.map_y_min = -cd.inf
            self.states.throttle_max = 0.5
            self.states.throttle_min = -0.5

    def calculate_yaw_from_quaternion(self, quaternion):
        # Function to convert Quaternions into Euler angles
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def send_control_actions_to_carla(self, control_actions):
        print("Error is : ")
        print(self.current_state)
        print(self.target_state)
        print(cd.norm_2(self.current_state - self.target_state))
        if ((cd.norm_2(self.current_state - self.target_state) > self.error_tolerance) and not(self.is_target_reached)):
            if (control_actions.acceleration > 0.0):
                self.vehicle_controls.throttle = control_actions.acceleration
                self.vehicle_controls.brake = 0.0
                # Divided by pi/6 to normalize the steering angle values to stay b/w [-1, 1]
                self.vehicle_controls.steer = control_actions.steering_angle / -(cd.pi/6)
                self.vehicle_controls.gear = 1
                self.vehicle_controls.hand_brake = False
                self.vehicle_controls.manual_gear_shift = False
                self.vehicle_controls.reverse = False
            elif (control_actions.acceleration < 0.0):
                self.vehicle_controls.throttle = control_actions.acceleration
                self.vehicle_controls.brake = 0.0
                self.vehicle_controls.steer = control_actions.steering_angle / (cd.pi/6)
                self.vehicle_controls.gear = 1
                self.vehicle_controls.hand_brake = False
                self.vehicle_controls.manual_gear_shift = False
                self.vehicle_controls.reverse = True
        else:
            self.vehicle_controls.brake = 1.0
            self.vehicle_controls.steer = 0.0
            self.vehicle_controls.throttle = 0.0
            self.vehicle_controls.hand_brake = True
            self.vehicle_controls.manual_gear_shift = False
            self.vehicle_controls.reverse = False
            self.is_target_reached = True
            self.get_logger().info("Target_Reached")

        self.mpc_iter = self.mpc_iter + 1
        print(self.mpc_iter)
        self.vehicle_control_publisher.publish(self.vehicle_controls)

    def ego_vehicle_status_callback(self, vehicle_status):
        try:
            self.ego_velocity = vehicle_status.velocity
            self.ego_orientaion_quaternion = vehicle_status.orientation
        except AttributeError:
            print("Please rerun the node, HINT : Callback Sync Failure")

    def obstacle_vehicle_status_callback(self, vehicle_status):
        try:
            self.obstacle_velocity = vehicle_status.velocity
            self.obstacle_orientaion_quaternion = vehicle_status.orientation
        except AttributeError:
            print("Please rerun the node, HINT : Callback Sync Failure")

    def control_action_callback(self, control_actions):
        self.send_control_actions_to_carla(control_actions)


def main(args=None):
    rclpy.init(args=args)
    carla_bridge_handler = CarlaBridgeHandler("Carla_Bridge_Handler_Node")
    rclpy.spin(carla_bridge_handler)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
