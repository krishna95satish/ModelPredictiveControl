import numpy as np

from frenet_optimal_trajectory import frenet_optimal_trajectory
from cubic_spline import cubic_spline_planner
from frenet_optimal_trajectory.frenet_optimal_trajectory import InitData
from trajectory_planning import utils


class TrajectoryPlanner(object):
    """
    Complete trajectory planner. Should eventually be responsible for all steps including
    route planning, behavior selector, etc. Currently only consists of motion planning.
    """

    def __init__(self, logger):
        self._logger = logger

    def plan_trajectory(self, init_data, waypoints, obstacles):
        return frenet_optimal_trajectory.plan_trajectory(init_data, waypoints, obstacles)

    def cartesian_to_frenet(self, x, y, waypoints, speed, vx, vy, ay):
        # waypoints
        wx = [point.x for point in waypoints]
        wy = [point.y for point in waypoints]
        #print("speed: ", speed, ", wx: ", waypoints)

        csp = cubic_spline_planner.Spline2D(wx, wy)
        s = csp.find_s(x, y)
        x_spline, y_spline = csp.calc_position(s)
        print("x, y, x_spline, y_spline, s: ", x, y, x_spline, y_spline, s)

        # get direction of point from spline
        # (https://math.stackexchange.com/questions/274712/calculate-on-which-side-of-a-straight-line-is-a-given-point-located)
        distance_spline_to_point = utils.distance(x, y, x_spline, y_spline)
        spline_to_point = utils.as_unit_vector(x_spline - x, y_spline - y, distance_spline_to_point)

        # normal spline vector
        x0, y0 = csp.calc_position(s)
        x1, y1 = csp.calc_position(s + 2)
        if x1 is None or y1 is None:
            return None
        spline_normal = y1 - y0, -(x1 - x0)
        spline_normal = utils.as_unit_vector(spline_normal[0], spline_normal[1])

        velocity_vector = vx, vy

        d = distance_spline_to_point * np.sign(utils.dot_product(spline_normal, spline_to_point))
        d_d = -speed * utils.dot_product(spline_normal, velocity_vector)
        d_dd = 0
        return InitData(s, d, speed, d_d, d_dd)
