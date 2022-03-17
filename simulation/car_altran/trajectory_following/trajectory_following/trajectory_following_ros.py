#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import math
import numpy as np

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point

#from custom_messages.msg import VehicleControl
#from custom_messages.msg import VehicleStatus
from custom_messages.msg import TrajectoryPoint
from custom_messages.msg import Trajectory
from carla_msgs.msg import CarlaEgoVehicleControl
from Components import qrgTrajectoryTracking
from Components import lrgTrajectoryTracking
from Components import lowpassfilter
from carla_msgs.msg import CarlaEgoVehicleStatus
from nav_msgs.msg import Odometry, Path
from transforms3d.euler import quat2euler



class TrajectoryFollowing(Node):
    def __init__(self):
        super().__init__("trajectory_following")
        # initial reference trajectory path
        pointtemp = TrajectoryPoint(x = 0.0, y = 0.0, yaw = 0.0, curvature = 0.0, velocity = 0.0, acceleration = 0.0)
        trajectorytemp_msg = Trajectory()
        trajectorytemp_msg.points.append(pointtemp)
        self.trajectorypath = trajectorytemp_msg

        # initial vehicle status
        self._velocity = 0.0
        self._odo = Odometry()

        # initial vehicle longitudinal and lateral controller
        self.Lrg = lrgTrajectoryTracking.Laengsregelung()
        self.Qrg = qrgTrajectoryTracking.Querregelung()

        # intial lowpass filter for ref. car velocity
        self._LP_vel_ref = lowpassfilter.LowpassConst( K=0.7, IV=0.0)

        # intial lowpass filter for throttle
        self._LP_throttle = lowpassfilter.LowpassConst( K=0.7, IV=0.0)

        # initial following ros topics
        self._following_publisher = self.create_publisher(CarlaEgoVehicleControl,
                                                          '/carla/altran/vehicle_control_cmd', 10)

        self._odometry_subscriber = self.create_subscription(Odometry, '/carla/altran/odometry',
                                                             self._odometry_update, 10)
        self._ego_vehicle_status_subscriber = self.create_subscription(CarlaEgoVehicleStatus,
                                                                       '/carla/altran/vehicle_status',
                                                                       self._ego_vehicle_status_callback, 10)

        self._trajectory_subscriber = self.create_subscription(Trajectory, '/aorta/trajectory/altran_internal',
                                                              self._publish_trajectory_path_update, 10)

        self.time = self.create_timer(0.1, self._odometry_publish_trajectory_following)

    def _odometry_publish_trajectory_following(self):
        msg = self._odo
        veh_x = msg.pose.pose.position.x
        veh_y = msg.pose.pose.position.y
        quaternion = (
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        )
        _, _, heading = quat2euler(quaternion)
        #_, _, heading = self._euler_from_quaternion(msg.pose.pose.orientation)
        #heading = 0.0
        veh_vel = self._velocity

        drive_command_status = False
        currtime_msg = self.get_clock().now().to_msg()
        currtime = currtime_msg.sec + currtime_msg.nanosec/1e9

        pathtime_start_msg = self.trajectorypath.header.stamp
        pathtime_start = pathtime_start_msg.sec + pathtime_start_msg.nanosec/1e9

        Path = self.trajectorypath.points
        IDpoint = 0
        for point in Path:
            pointtime = pathtime_start + point.stamp.sec + point.stamp.nanosec/1e9
            IDpoint = IDpoint + 1
            if pointtime >= currtime + 0.3:
                x_ref = point.x
                y_ref = point.y
                yaw_ref = point.yaw
                #yaw_ref = 0.0
                vel_ref = point.velocity
                acc_ref = point.acceleration
                drive_command_status = True
                self.get_logger().info('Traj. length= {}'.format(len(Path)))
                self.get_logger().info('Ref. Point ID = {}'.format(IDpoint))
                self.get_logger().info('Ref. velocity = {}'.format(vel_ref))
                self.get_logger().info('Curr velocity = {}'.format(veh_vel))
                self.get_logger().info('Curr. time = {}'.format(currtime))
                self.get_logger().info('Ref. time = {}'.format(pointtime))
                break

        # brake moment active over trajectory path
        if not drive_command_status:
            a_fr = -3*veh_vel
            a_fr = np.clip(a_fr, -3.0, 3.0)

            EgoVehi_msg = CarlaEgoVehicleControl()

            if a_fr > 0:
                throttle = a_fr/3
                brake = 0.0
                reverse = bool(0)
                gear = 1
            else:
                throttle = 0.0
                brake = -a_fr/10
                #brake = 0.0
                reverse = bool(0)
                gear = 0

            EgoVehi_msg.hand_brake = True
            manual_gear_shift = bool(0)
            EgoVehi_msg.throttle = throttle
            EgoVehi_msg.steer = 0.0
            EgoVehi_msg.brake = brake
            EgoVehi_msg.reverse = reverse
            EgoVehi_msg.gear = gear
            EgoVehi_msg.manual_gear_shift = manual_gear_shift
            self._following_publisher.publish(EgoVehi_msg)
            self.get_logger().info('Publishing vehicle control to stop.')

        # follow trajectory path
        if drive_command_status:
            veh_yaw = heading
            if veh_yaw > math.pi:
                veh_yaw = veh_yaw - 2*math.pi
            if veh_yaw < -math.pi:
                veh_yaw = veh_yaw + 2*math.pi
            
            self.get_logger().info('Vel. heading= {}'.format(veh_yaw ))
            vel_ref = self._LP_vel_ref.run_step(vel_ref)

            Curvature = self.Qrg.qrgBfr(x_ref, y_ref, yaw_ref, veh_x, veh_y, veh_yaw)
            steering = math.atan(3*Curvature)
            steering = np.clip(steering, -30/180*math.pi, 30/180*math.pi)

            a_fr = self.Lrg.lrgBfr(x_ref, y_ref, yaw_ref, vel_ref, veh_x, veh_y, veh_vel)
            a_fr = a_fr + acc_ref
            a_fr = np.clip(a_fr, -5.0, 5.0)

            EgoVehi_msg = CarlaEgoVehicleControl()

            # set drive command to carla vehicle
            if a_fr > 0:
                throttle = a_fr/5
                brake = 0.0
                reverse = bool(0)
                gear = 1
            else:
                throttle = a_fr/5*0
                #brake = -a_fr/10
                brake = 0.0
                reverse = bool(0)
                gear = 1
            
            # add throttle according to tire resistance 
            # if throttle >= 0:
            #     throttle = throttle

            throttle = np.clip(throttle, 0.0, 1.0)
            throttle = self._LP_throttle.run_step(throttle)
            steer = steering/(3/18*math.pi)

            manual_gear_shift = False

            EgoVehi_msg.throttle = throttle
            EgoVehi_msg.steer = steer
            EgoVehi_msg.brake = brake
            EgoVehi_msg.hand_brake = False
            EgoVehi_msg.reverse = reverse
            #EgoVehi_msg.gear = gear
            EgoVehi_msg.manual_gear_shift = manual_gear_shift
            self._following_publisher.publish(EgoVehi_msg)
            self.get_logger().info('Publishing vehicle control for trajectory following.')


    def _publish_trajectory_path_update(self, trajectory_altran):
        self.trajectorypath = trajectory_altran
        self.get_logger().info('Received altran vehicle trajectory.')

    def _ego_vehicle_status_callback(self, msg):
        self._velocity = msg.velocity  # m/s
        self.get_logger().info('Received altran vehicle velocity.')

    def _odometry_update(self, msg):
        self._odo= msg
        self.get_logger().info('Received altran vehicle odometry.')
        
    def _euler_from_quaternion(self, quaternion):
    	"""
    	Converts quaternion to euler roll, pitch, yaw
    	quaternion = [x, y, z, w]
   	    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
   	    """
    	x = quaternion.x
    	y = quaternion.y
    	z = quaternion.z
    	w = quaternion.w

    	sinr_cosp = 2 * (w * x + y * z)
    	cosr_cosp = 1 - 2 * (x * x + y * y)
    	roll = np.arctan2(sinr_cosp, cosr_cosp)

    	sinp = 2 * (w * y - z * x)
    	pitch = np.arcsin(sinp)

    	siny_cosp = 2 * (w * z + x * y)
    	cosy_cosp = 1 - 2 * (y * y + z * z)
    	yaw = np.arctan2(siny_cosp, cosy_cosp)

    	return roll, pitch, yaw



def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryFollowing()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
