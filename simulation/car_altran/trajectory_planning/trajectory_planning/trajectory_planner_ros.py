import rclpy
from rclpy.node import Node

import math
import numpy as np

from std_msgs.msg import Float64
from ros_compatibility import QoSProfile
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry, Path
from builtin_interfaces.msg import Time
from custom_messages.msg import TrajectoryPoint
from custom_messages.msg import Trajectory
from custom_messages.msg import Trajectories
from carla_msgs.msg import CarlaEgoVehicleStatus

from trajectory_planning import trajectory_planner
from trajectory_planning import utils


class TrajectoryPlannerRos(Node):
    def __init__(self):
        super().__init__('trajectory_planner', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self._waypoints_subscriber = self.create_subscription(Trajectory, '/aorta/trajectory/filtered/altran',
                                                              self._waypoints_callback, 10)
        self._odometry_subscriber = self.create_subscription(Odometry, '/carla/altran/odometry',
                                                             self._odometry_callback, 10)
        self._ego_vehicle_status_subscriber = self.create_subscription(CarlaEgoVehicleStatus,
                                                                       '/carla/altran/vehicle_status',
                                                                       self._ego_vehicle_status_callback, 10)

        # self._waypoint_publisher = self.create_publisher(Path, '/carla/altran/waypoints_input', 10)
        # self._path_publisher = self.create_publisher(Path, '/carla/altran/waypoints',
        #                                              QoSProfile(depth=1, durability=True))
        # self._waypoint_speed_publisher = self.create_publisher(Float64, '/carla/altran/speed_command',
        #                                                        QoSProfile(depth=1, durability=True))
        # self._current_pose_publisher = self.create_publisher(PoseStamped, '/current_position', 10)

        self._trajectory_publisher = self.create_publisher(Trajectory, '/aorta/trajectory/altran_internal', 10)
        self._trajectories_publisher = self.create_publisher(Trajectories, '/aorta/trajectory/altran_all', 10)

        self.trajectory_planner = trajectory_planner.TrajectoryPlanner(self.get_logger())
        self.initiated = False
        self.waypoints = []
        self._velocity = 0.0
        self._accel_y = 0.0
        # temporary obstacle lists. Should be received as subscription
        # self.obstacles = np.array([[20.0, 10.0],
        #                           [30.0, 6.0],
        #                           [30.0, 8.0],
        #                           [35.0, 8.0],
        #                           [50.0, 3.0]])
        self.obstacles = []

        self._publish_all_trajs = self.get_parameter('publish_all_trajectories').get_parameter_value().bool_value

    def _run_planner(self, init_data):
        trajectory, all_trajectories = self.trajectory_planner.plan_trajectory(init_data, self.waypoints,
                                                                               self.obstacles)

        if trajectory is None:
            self.get_logger().info("No viable trajectory generated..")
            return False

        # # path msg only for visualizing with Rviz
        # path_msg = Path()
        # path_msg.header.frame_id = "map"
        # path_msg.header.stamp = self.get_clock().now().to_msg()

        trajectory_msg = self._generate_empty_trajectory_msg()
        self._fill_trajectory_msg(trajectory, trajectory_msg)

        # self._path_publisher.publish(path_msg)
        self._trajectory_publisher.publish(trajectory_msg)

        if self._publish_all_trajs:
            print("publish all!")
            trajectories_msg = self._generate_empty_trajectories_msg()
            self._fill_trajectories_msg(all_trajectories, trajectories_msg)
            self._trajectories_publisher.publish(trajectories_msg)
        print("No of trajectories: ", len(all_trajectories))
        print("No of points in ref trajectory: ", len(trajectory.t))

        # target speed only needed by carla local planner
        # target_speed = Float64()
        # target_speed.data = trajectory.s_d[-1] * 3.6  # local_planner from carla uses kmph
        # print("target speed: ", target_speed.data)
        # self._waypoint_speed_publisher.publish(target_speed)

        # temp solution for showing pose in Rviz
        # current_pose = PoseStamped()
        # current_pose.header.frame_id = "map"
        # current_pose.pose.position = Point(x=trajectory.x[0], y=trajectory.y[0])
        # quaternion = utils.quaternion_from_euler(0, 0, -trajectory.yaw[1])
        # current_pose.pose.orientation = Quaternion(w=quaternion[0], x=quaternion[1], y=quaternion[2],
        #                                            z=quaternion[3])
        # self._current_pose_publisher.publish(current_pose)

    def _waypoints_callback(self, waypoints):
        self.waypoints = waypoints.points
        # msg = self._get_path_from_trajectory(waypoints)
        # self._waypoint_publisher.publish(msg)

    def _odometry_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        _, _, heading = utils.euler_from_quaternion(msg.pose.pose.orientation)
        vx = self._velocity * math.cos(heading)
        vy = self._velocity * math.sin(heading)
        if self.waypoints:
            init_data = self.trajectory_planner.cartesian_to_frenet(x, y, self.waypoints, self._velocity, vx, vy,
                                                                    self._accel_y)
            if init_data is not None:
                self._run_planner(init_data)
            else:
                self.get_logger().info("goal position reached..")
        else:
            self.get_logger().info("waiting for waypoints..")

    def _ego_vehicle_status_callback(self, msg):
        self._velocity = msg.velocity  # m/s
        # self._accel_x = msg.acceleration.linear.x
        self._accel_y = msg.acceleration.linear.y

    def _get_path_from_trajectory(self, trajectory):
        # TODO: also use other values
        path = Path()
        for point in trajectory.points:
            pose = PoseStamped()
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = 0.0

            path.poses.append(pose)

        path.header = trajectory.header

        return path

    def _fill_trajectory_msg(self, trajectory, trajectory_msg: Trajectory):
        for t, x, y, yaw, curv, s, s_d, s_dd, d, d_d, d_dd in zip(trajectory.t, trajectory.x, trajectory.y,
                                                                  trajectory.yaw, trajectory.c, trajectory.s,
                                                                  trajectory.s_d, trajectory.s_dd, trajectory.d,
                                                                  trajectory.d_d, trajectory.d_dd):
            # pose = PoseStamped()
            # pose.pose.position = Point(x=x, y=y)
            # path_msg.poses.append(pose)

            t_s = math.floor(t)
            t_ns = (t % 1) * 1e9
            stamp = Time(sec=t_s, nanosec=int(t_ns))
            point = TrajectoryPoint(stamp=stamp, x=x, y=y, yaw=yaw, curvature=curv, velocity=s_d, acceleration=s_dd)
            trajectory_msg.points.append(point)
            trajectory_msg.cost = trajectory.cf

    def _fill_trajectories_msg(self, all_trajectories, trajectories_msg: Trajectories):
        for trajectory in all_trajectories:
            trajectory_msg = self._generate_empty_trajectory_msg()
            self._fill_trajectory_msg(trajectory, trajectory_msg)
            trajectories_msg.trajectories.append(trajectory_msg)

    def _generate_empty_trajectory_msg(self):
        trajectory_msg = Trajectory()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg.header.frame_id = "map"
        return trajectory_msg

    def _generate_empty_trajectories_msg(self):
        trajectories_msg = Trajectories()
        trajectories_msg.header.stamp = self.get_clock().now().to_msg()
        trajectories_msg.header.frame_id = "map"
        return trajectories_msg


def main(args=None):
    rclpy.init(args=args)

    trajectory_planner_ros = TrajectoryPlannerRos()

    rclpy.spin(trajectory_planner_ros)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    trajectory_planner_ros.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
