#!/usr/bin/env python3
import ad_map_access as ad
from numpy import size
import matplotlib.pyplot as plt
from scipy import interpolate
import csaps
from sqlalchemy import true

class LaneExtractor:
    def __init__(self) -> None:
        print("Lane Extractoion Running.....")
        # Map related methods
        ad.map.access.init("/home/krishna/Desktop/Git_MPC_Python/trajectory_gen_mpc/src/carla_map_access/Town05.txt")
        self.map_matching = ad.map.match.AdMapMatching()
        self.ego_pose_point = ad.map.point.ENUPoint()
        self.goal_pose_point = ad.map.point.ENUPoint()
        self.physical_distance = 0.5
        self.physical_probability = 0.1
        self.goal_lane_ID = 140151
        self.ego_lane_ID = None
    
    def set_ego_position(self, ego_x, ego_y, ego_z):
        self.ego_pose_point.x = ego_x
        self.ego_pose_point.y = ego_y
        self.ego_pose_point.z = ego_z
    
    def set_goal_position(self, goal_x, goal_y, goal_z):
        self.goal_pose_point.x = goal_x
        self.goal_pose_point.y = goal_y
        self.goal_pose_point.z = goal_z

    def ego_localization(self):
        min_length = float('inf')
        localization_data = self.map_matching.getMapMatchedPositions(self.ego_pose_point, ad.physics.Distance(self.physical_distance), ad.physics.Probability(self.physical_probability))
        if not localization_data:
            print("WARNING: Couldn't find a parapoint for location '{}'.".format(self.ego_pose_point))
        goal_point_data = self.map_matching.getMapMatchedPositions(self.goal_pose_point, ad.physics.Distance(self.physical_distance), ad.physics.Probability(self.physical_probability))
        if not goal_point_data:
            print("WARNING: Couldn't find a parapoint for location '{}'.".format(self.goal_pose_point))
        for start_match in localization_data:
            start_point = start_match.lanePoint.paraPoint
            for end_match in goal_point_data:
                end_point = end_match.lanePoint.paraPoint

                # Get the route
                new_route_segment = ad.map.route.planRoute(start_point, end_point)
                edgeList = ad.map.route.getGeoBorderOfRoute(new_route_segment)
                for each in edgeList:
                    for each_right in each.right:
                        print(each_right.latitude)
                        print(each_right.longitude)
                        print("######")
                if len(new_route_segment.roadSegments) == 0:
                    continue  # The route doesn't exist, ignore it

                # Calculate route length, as a sum of the mean of the road's lanes length
                length = 0
                for road_segment in new_route_segment.roadSegments:
                    road_length = 0
                    number_lanes = 0
                    for lane_segment in road_segment.drivableLaneSegments:
                        seg_start = float(lane_segment.laneInterval.start)
                        seg_end = float(lane_segment.laneInterval.end)
                        seg_length = float(ad.map.lane.calcLength(lane_segment.laneInterval.laneId))

                        road_length += seg_length * abs(seg_end - seg_start)
                        number_lanes += 1

                    if number_lanes != 0:
                        length += road_length / number_lanes

                # Save the shortest route
                if length < min_length:
                    min_length = length
                    route_segment = new_route_segment
                    start_lane_id = start_point.laneId
                    #self.get_route_lane_list(route_segment,start_lane_id)

        if not route_segment:
            print("WARNING: Couldn't find a viable route between locations ")

    def get_route_lane_list(self, route, start_lane_id, prev_lane_id=None):
        """Given a route and its starting lane, returns the lane segments corresponding
        to the route. This supposes that no lane changes occur during the route"""
        segments = []

        for road_segment in route.roadSegments:
            for lane_segment in road_segment.drivableLaneSegments:

                if prev_lane_id and prev_lane_id not in lane_segment.predecessors:
                    continue  # Lane doesn't connect to the previous one
                elif not prev_lane_id and lane_segment.laneInterval.laneId != start_lane_id:
                    continue  # Lane is different than the starting one
                prev_lane_id = lane_segment.laneInterval.laneId

                segments.append(lane_segment)

        return segments

    def inperpolate_boundaries(self):
        y = [35.4, 36.3]
        x = [104.3, 135.4]
        f = interpolate.interp1d(x, y, fill_value="extrapolate")
        print(f(35.9))
    #def interpolate_map_boundaries(self):
    #    self.interpolate_right_lane = self.inperpolate_boundaries(
    #        self.x, self.y_right)
    #    self.interpolate_left_lane = self.inperpolate_boundaries(
    #        self.x, self.y_left)
#
#
    #def extract_boundaies(self, map_boundary):
    #    return self.interpolate_right_lane(map_boundary + self.offset), self.interpolate_left_lane(map_boundary + self.offset)
#
    #def visualize_interpolations(self):
    #    plt.plot(self.x, self.y_right, '-o')
    #    plt.plot(self.x, self.y_left, '-o')
    #    plt.show()


def main():
    freeSpaceServer = LaneExtractor()
    freeSpaceServer.set_ego_position(34.5, 128.0, 0.0)
    freeSpaceServer.set_goal_position(68.4885, 145.38, 0.0)
    freeSpaceServer.ego_localization()
    freeSpaceServer.inperpolate_boundaries()

if __name__ == "__main__":
    main()
