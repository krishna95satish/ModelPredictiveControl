import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from custom_messages.msg import TrajectoryPoint
from custom_messages.msg import Trajectory
from custom_messages.msg import TrajectoryDataPoint
from custom_messages.msg import TrajectoryData


class DataGenerator(Node):
    def __init__(self):
        super().__init__('dummy_data_generator')

        self._waypoints_publisher_rviz = self.create_publisher(Path, '/aorta/trajectory/altran_rviz', 10)
        self._waypoints_publisher = self.create_publisher(Trajectory, '/aorta/trajectory/altran', 10)

        # temporary information needed by trajectory planner
        self._initial_pos_publisher = self.create_publisher(TrajectoryData, '/initial_data', 10)
        self._trajectory_data_subscriber = self.create_subscription(TrajectoryData, '/trajectory_data',
                                                                    self._trajectory_data_callback, 10)

        timer_period = 3.0  # seconds
        self.timer = self.create_timer(timer_period, self._publish_waypoints)
        self.positions = [Point(x=0.0, y=0.0),
                          Point(x=10.0, y=-6.0),
                          Point(x=20.5, y=5.0),
                          Point(x=35.0, y=6.5),
                          Point(x=70.5, y=0.0),
                          Point(x=85.0, y=0.5)]

    def _trajectory_data_callback(self, trajectory_data):
        msg = TrajectoryData()
        if len(trajectory_data.points) < 2:
            print("No more points")
        else:
            # send the next position as new initial position
            next_init_point = trajectory_data.points[1]
            msg.points.append(next_init_point)
            self._initial_pos_publisher.publish(msg)

    def _publish_waypoints(self):
        # path msg only for visualizing with Rviz
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        trajectory_msg = Trajectory()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg.header.frame_id = "map"

        for position in self.positions:
            pose = PoseStamped()
            pose.pose.position = position

            # quaternion = tf.transformations.quaternion_from_euler(
            #     0, 0, -math.radians(wp[0].transform.rotation.yaw))
            # pose.pose.orientation.x = quaternion[0]
            # pose.pose.orientation.y = quaternion[1]
            # pose.pose.orientation.z = quaternion[2]
            # pose.pose.orientation.w = quaternion[3]

            path_msg.poses.append(pose)

            point = TrajectoryPoint(x=position.x, y=position.y)
            trajectory_msg.points.append(point)

        self._waypoints_publisher.publish(trajectory_msg)  # to be published by aorta server
        self._waypoints_publisher_rviz.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)

    waypoints_generator = DataGenerator()

    rclpy.spin(waypoints_generator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    waypoints_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
