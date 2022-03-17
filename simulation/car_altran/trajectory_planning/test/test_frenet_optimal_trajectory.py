import pytest
import numpy as np

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
# from custom_messages.msg import TrajectoryDataPoint

from frenet_optimal_trajectory import frenet_optimal_trajectory


def test_frenet_optimal_trajectory():
    positions = [Point(x=0.0, y=0.0),
                 Point(x=10.0, y=-6.0),
                 Point(x=20.5, y=5.0),
                 Point(x=35.0, y=6.5),
                 Point(x=70.5, y=0.0)]
    path = Path()
    for position in positions:
        pose = PoseStamped()
        pose.pose.position = position

        path.poses.append(pose)
    # obstacle lists
    obstacles = np.array([[20.0, 10.0],
                          [30.0, 6.0],
                          [30.0, 8.0],
                          [35.0, 8.0],
                          [50.0, 3.0]])

    # initial state
    c_speed = 10.0 / 3.6  # current speed [m/s]
    c_d = 2.0  # current lateral position [m]
    c_d_d = 0.0  # current lateral speed [m/s]
    c_d_dd = 0.0  # current lateral acceleration [m/s]
    s0 = 0.0  # current course position [m]
    # init_point = TrajectoryDataPoint(s=s0, s_d=c_speed, d=c_d, d_d=c_d_d, d_dd=c_d_dd)

    trajectory = frenet_optimal_trajectory.plan_trajectory(path.poses, obstacles)
    assert (len(trajectory.x) == 25)


if __name__ == '__main__':
    test_frenet_optimal_trajectory()
