#!/usr/bin/env python3
from casadi import pi, inf
from time import time
import numpy as np
import casadi as cd
from trajectory_gen_mpc.GlobalConstraints import VehicleConstraints, SolverOptions, MpcTunableParameters, StaticObstacleParameters, DynamicObstacleParameters
from trajectory_gen_mpc.NonHolonomic import NonHolonomic
from trajectory_gen_mpc.MultipleShooting import MultipleShooting
from trajectory_gen_mpc.StaticObstacle import StaticObstacle
from trajectory_gen_mpc.DynamicObstacle import DynamicObstacle

def DM2Arr(dm):
    return np.array(dm.full())

def main():

    vehicle_constrinats = VehicleConstraints(
        0.5, 0.1, -0.1, (pi/6), -(pi/6), inf, -inf, 5.0, -5.0, inf, -inf, inf, -inf, 0.3)
    solver_options = SolverOptions(1000, 0, 1e-3, 1e-3, 0)
    mpc_tunable_parameters = MpcTunableParameters(
        30, 20, 20, 3, 18, 50, 215, 0, 0.3)
    static_obstacle1_parameter = StaticObstacleParameters(2.0, 0.8, 0.6)
    dynamic_obstacle1_parameter = DynamicObstacleParameters(30.0, -20.0, 0.6, 0.3, 0.0, mpc_tunable_parameters.Time_delta)
    obstacle_list = []
    static_obstacle1 = StaticObstacle(static_obstacle1_parameter)
    #static_obstacle2 = StaticObstacle(static_obstacle1_parameter)
    dynamic_obstacle1 = DynamicObstacle(dynamic_obstacle1_parameter)
    obstacle_list.append(static_obstacle1)
    obstacle_list.append(dynamic_obstacle1)
    # obstacle_list.append(static_obstacle2)
    car_altran = NonHolonomic(vehicle_constrinats)

    multiple_shooting_solver = MultipleShooting(car_altran, obstacle_list)
    multiple_shooting_solver.init_solver(
        solver_options, mpc_tunable_parameters)
    solver = multiple_shooting_solver.get_solver()

    # main loop for solver testing

    x_init = 0.0
    y_init = 0.0
    v_init = 0.0
    theta_init = 0.0

    x_target = 5.0
    y_target = 2.5
    v_target = 0.0
    theta_target = 0.0

    state_init = cd.DM([x_init, y_init, v_init, theta_init]
                       )            # initial state
    state_target = cd.DM([x_target, y_target, v_target,
                         theta_target])  # target state
    u0 = cd.DM.zeros((car_altran.get_num_control(),
                     mpc_tunable_parameters.prediction_horizon))  # initial control

    # initial state full
    X0 = cd.repmat(state_init, 1, mpc_tunable_parameters.prediction_horizon+1)

    lbg, ubg = multiple_shooting_solver.get_constraints()
    lbx, ubx = car_altran.get_constraints()

    args = {
        'lbg': lbg,  # constraints lower bound
        'ubg': ubg,  # constraints upper bound
        'lbx': lbx,
        'ubx': ubx
    }
    sim_time = 20
    mpc_iter = 0
    t0 = 0
    cat_states = DM2Arr(X0)
    cat_controls = DM2Arr(u0[:, 0])
    t = cd.DM(t0)
    times = np.array([[0]])

    ################################

    while (cd.norm_2(state_init - state_target) > 0.105) and (mpc_iter * mpc_tunable_parameters.Time_delta < sim_time):
        t1 = time()
        args['p'] = cd.vertcat(
            state_init,    # current state
            state_target   # target state
        )
        # optimization variable current state
        args['x0'] = cd.vertcat(
            cd.reshape(X0, car_altran.get_num_states() *
                       (mpc_tunable_parameters.prediction_horizon+1), 1),
            cd.reshape(u0, car_altran.get_num_control()
                       * mpc_tunable_parameters.prediction_horizon, 1)
        )
        sol = solver(
            x0=args['x0'],
            lbx=args['lbx'],
            ubx=args['ubx'],
            lbg=args['lbg'],
            ubg=args['ubg'],
            p=args['p']
        )

        u = cd.reshape(sol['x'][car_altran.get_num_states() *
                       (mpc_tunable_parameters.prediction_horizon + 1):], car_altran.get_num_control(), mpc_tunable_parameters.prediction_horizon)
        X0 = cd.reshape(sol['x'][: car_altran.get_num_states()
                        * (mpc_tunable_parameters.prediction_horizon+1)], car_altran.get_num_states(), mpc_tunable_parameters.prediction_horizon+1)

        cat_states = np.dstack((
            cat_states,
            DM2Arr(X0)
        ))
        cat_controls = np.vstack((
            cat_controls,
            DM2Arr(u[:, 0])
        ))
        t = np.vstack((
            t,
            t0
        ))


        t0, state_init, u0 = car_altran.apply_first_control_action(
            state_init, u, mpc_tunable_parameters.Time_delta, t0)

        X0 = cd.horzcat(
            X0[:, 1:],
            # reinitializing with the predicted state form solver
            cd.reshape(X0[:, -1], -1, 1)
        )
        t2 = time()
        times = np.vstack((
            times,
            t2-t1
        ))
        mpc_iter = mpc_iter + 1
        print(state_init)



if __name__ == "__main__":
    main()
