#!/usr/bin/env python3
import casadi as cd
import rclpy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from casadi import pi, inf
from custom_interfaces.msg import States, ControlAction, TrajectoryPoint, Trajectory
from trajectory_gen_mpc.GlobalConstraints import VehicleConstraints
from trajectory_gen_mpc.NonHolonomic import NonHolonomic
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class CarlaHandler(Node):
    def __init__(self, node_name):
        super().__init__(node_name=node_name)
        self.state_publisher = self.create_publisher(States, "Vehicle_states", 10)
        self.create_subscription(ControlAction, "Control_actions", self.control_action_callback, 10)
        self.waypoints_publisher_rviz = self.create_publisher(Path, '/aorta/trajectory/altran_rviz', 10)
        self.waypoints_publisher = self.create_publisher(Trajectory, '/aorta/trajectory/altran', 10)
        self.timer = self.create_timer(0.1, self.state_publisher_callback)
        ######################## Vizualization configuration #############################
        self.marker_publisher = self.create_publisher(Marker, "Rviz_marker", 10)
        self.trajectory_line_marker = Marker()
        self.initial_marker = Marker()
        self.obstacle_marker = Marker()
        self.final_marker = Marker()
        self.trajectory_msg = Trajectory()
        self.path_msg = Path()
        ######################## MPC related configurations ##############################
        vehicle_constrinats = VehicleConstraints(0.5, 0.1, -0.1, (pi/6), -(pi/6), inf, -inf, 5.0, -5.0, inf, -inf, inf, -inf, 0.3)
        car = NonHolonomic(vehicle_constrinats)
        car.create_model()
        self.non_linear_fun_generator = car.get_model()

        self.mpc_iter = 0
        self.delta_T = 0.3
        self.sim_time = 20
        self.error_tolerance = 0.205
        self.is_target_reached = False
        self.next_state = None

        self.get_logger().info("Carla Node initialized")
        self.state_initializer()


    def state_initializer(self):
        self.states = States()
        self.x_i = self.states.initial_state_x = 0.0
        self.y_i = self.states.initial_state_y = 0.0
        self.states.initial_state_v = 0.0
        self.states.initial_state_theta = 0.0

        self.x_f = self.states.final_state_x = 5.0
        self.y_f = self.states.final_state_y = 2.5
        self.states.final_state_v = 0.0
        self.states.final_state_theta = 0.0

        self.current_state = cd.DM([self.states.initial_state_x,
                                    self.states.initial_state_y,
                                    self.states.initial_state_v,
                                    self.states.initial_state_theta]
                                   )
        self.target_state = cd.DM([self.states.final_state_x,
                                   self.states.final_state_v,
                                   self.states.final_state_y,
                                   self.states.final_state_theta]
                                  )

    def state_publisher_callback(self):
        if (not(self.is_target_reached)):
            self.state_publisher.publish(self.states)
            self.marker_publisher.publish(self.initial_marker)
            self.marker_publisher.publish(self.final_marker)
            self.marker_publisher.publish(self.obstacle_marker)
            self.marker_publisher.publish(self.trajectory_line_marker)
            self.waypoints_publisher.publish(self.trajectory_msg)  # to be published by aorta server
            self.waypoints_publisher_rviz.publish(self.path_msg)
        else:
            # Do nothing
            pass

    def control_action_callback(self, control_actions):
        if (cd.norm_2(self.current_state - self.target_state) > self.error_tolerance) and (self.mpc_iter * self.delta_T < self.sim_time):

            self.current_state = self.dummy_car_model(control_actions.acceleration, 
                                                      control_actions.steering_angle)

            self.states.initial_state_x = float(self.current_state[0])
            self.states.initial_state_y = float(self.current_state[1])
            self.states.initial_state_v = float(self.current_state[2])
            self.states.initial_state_theta = float(self.current_state[3])

            self.mpc_iter = self.mpc_iter + 1
        else:
            self.is_target_reached = True
            print("Target reached")

    
    def dummy_car_model(self, acceleration, steering_angle):
        # Constructing a model which represents our car in carla
        temp_controls = cd.vertcat(acceleration, steering_angle)
        self.next_state = cd.DM.full(self.current_state + (self.delta_T * self.non_linear_fun_generator(self.current_state, temp_controls)))
        self.plotRviz(self.next_state)
        return self.next_state
    
    def plotRviz(self, state):
        self.trajectory_line_marker.header.frame_id = "/MPC_trajectories"
        self.initial_marker.header.frame_id = "/MPC_trajectories"
        self.obstacle_marker.header.frame_id = "/MPC_trajectories"
        self.final_marker.header.frame_id = "/MPC_trajectories"

        self.trajectory_line_marker.header.stamp = self.get_clock().now().to_msg()
        self.initial_marker.header.stamp = self.get_clock().now().to_msg()
        self.obstacle_marker.header.stamp = self.get_clock().now().to_msg()
        self.final_marker.header.stamp = self.get_clock().now().to_msg()

        self.trajectory_line_marker.ns = "mpc_points"
        self.initial_marker.ns = "mpc_points"
        self.obstacle_marker.ns = "mpc_points"
        self.final_marker.ns = "mpc_points"

        self.trajectory_line_marker.action = Marker.ADD
        self.initial_marker.action = Marker.ADD
        self.obstacle_marker.action = Marker.ADD
        self.final_marker.action = Marker.ADD

        self.trajectory_line_marker.id = 0
        self.initial_marker.id = 1
        self.obstacle_marker.id = 2
        self.final_marker.id = 3

        self.trajectory_line_marker.type = Marker.LINE_STRIP
        self.initial_marker.type = Marker.CUBE
        self.obstacle_marker.type = Marker.SPHERE
        self.final_marker.type = Marker.CUBE

        self.trajectory_line_marker.scale.x = 0.05
        self.initial_marker.scale.x = 0.5
        self.initial_marker.scale.y = 0.5
        self.initial_marker.scale.z = 0.5
        self.obstacle_marker.scale.x = 0.5
        self.obstacle_marker.scale.y = 0.5
        self.obstacle_marker.scale.z = 0.5
        self.final_marker.scale.x = 0.5
        self.final_marker.scale.y = 0.5
        self.final_marker.scale.z = 0.5

        self.trajectory_line_marker.color.r = 1.0
        self.trajectory_line_marker.color.a = 1.0

        self.initial_marker.color.g = 1.0
        self.initial_marker.color.a = 1.0

        self.obstacle_marker.color.r = 1.0
        self.obstacle_marker.color.a = 1.0

        self.final_marker.color.g = 1.0
        self.final_marker.color.a = 1.0

        self.initial_marker.pose.position.x = self.x_i
        self.initial_marker.pose.position.y = self.y_i
        self.initial_marker.pose.orientation.w = 1.0

        self.obstacle_marker.pose.position.x = 2.0     # Temp for vizualizations_x 
        self.obstacle_marker.pose.position.y = 0.8     # Temp for vizualizations_y
        self.obstacle_marker.pose.orientation.w = 1.0

        self.final_marker.pose.position.x = self.x_f
        self.final_marker.pose.position.y = self.y_f
        self.final_marker.pose.orientation.w = 1.0

        point = Point()
        point.x = float(state[0])
        point.y = float(state[1])
        point.z = 0.2
        self.trajectory_line_marker.pose.orientation.w = 1.0
        self.trajectory_line_marker.points.append(point)
        ############ Feeding Plugin #####################
        self.path_msg.header.frame_id = "map"
        self.path_msg.header.stamp = self.get_clock().now().to_msg()

        self.trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        self.trajectory_msg.header.frame_id = "map"

        pose = PoseStamped()
        pose.pose.position.x = float(state[0])
        pose.pose.position.y = float(state[1])
        self.path_msg.poses.append(pose)
        point = TrajectoryPoint(x=float(state[0]), y=float(state[1]))
        self.trajectory_msg.points.append(point)


def main(args=None):
    rclpy.init(args=args)
    carla_handler = CarlaHandler("carla_node")
    rclpy.spin(carla_handler)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
