#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate


class DummyFreeSpaceServer:
    def __init__(self) -> None:
        print("FreeSpace Server Initialized")
        # range is -200 .. 200
        self.map_length = 400
        self.x = np.arange(self.map_length, dtype=float)
        self.offset = 200
        # total entries is mod(200) + mod(200) = 400
        # right_side_lane
        self.lane_right_open_idx = -141.0
        self.lane_right_close_idx = -113.0
        self.normal_right_lane_boundary = -91.8
        self.right_hand_turn_boundary = -150
        self.y_right = np.arange(self.map_length, dtype=float)
        # left_side_lane
        self.lane_left_open_idx = -141.0
        self.lane_left_close_idx = -113.0
        self.left_hand_turn_boundary = 50
        self.normal_left_lane_boundary = -90.0
        self.y_left = np.arange(self.map_length, dtype=float)
        self.interpolate_right_lane = None
        self.interpolate_left_lane = None

    def create_dummy_map(self):
        for i in range(self.map_length):
            if ((i < self.lane_right_open_idx + self.offset) or i > (self.lane_right_close_idx + self.offset)):
                self.y_right[i] = self.normal_right_lane_boundary
                self.y_left[i] = self.normal_left_lane_boundary
            else:
                self.y_right[i] = self.right_hand_turn_boundary
                self.y_left[i] = self.left_hand_turn_boundary

    def interpolate_map_boundaries(self):
        self.interpolate_right_lane = self.inperpolate_boundaries(
            self.x, self.y_right)
        self.interpolate_left_lane = self.inperpolate_boundaries(
            self.x, self.y_left)

    def inperpolate_boundaries(self, x, y):
        return interpolate.interp1d(x, y, fill_value="extrapolate")

    def extract_boundaies(self, map_boundary):
        return self.interpolate_right_lane(map_boundary + self.offset), self.interpolate_left_lane(map_boundary + self.offset)

    def visualize_interpolations(self):
        plt.plot(self.x, self.y_right, '-o')
        plt.plot(self.x, self.y_left, '-o')
        plt.show()


def main():
    freeSpaceServer = DummyFreeSpaceServer()
    freeSpaceServer.create_dummy_map()
    freeSpaceServer.interpolate_map_boundaries()
    freeSpaceServer.visualize_interpolations()
    print(freeSpaceServer.extract_boundaies(-128)[1])


if __name__ == "__main__":
    main()
