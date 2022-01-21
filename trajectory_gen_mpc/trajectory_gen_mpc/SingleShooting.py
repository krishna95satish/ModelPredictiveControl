#!/usr/bin/env python3
from trajectory_gen_mpc.Solver import Solver


class SingleShooting(Solver):
    def __init__(self) -> None:
        super().__init__()

    def set_options(self, options):
        pass

    def set_constraints(self):
        pass

    def set_objectives(self):
        pass
