import numpy as np
import matplotlib.pyplot as plt

x = [32.04, 32.44]
y = [104.3, 135.5]

x_new = 120.0

y_new = np.interp(x_new, y, x)
print(y_new)
# 13.0

plt.plot(x, y, "og-", x_new, y_new, "or")
plt.show()



            ### For testing the goal point must be in the ego's lane
            #temp_ego_lane_ID = ad.map.lane.getLane(tempLaneID.lanePoint.paraPoint.laneId)
            #print(temp_ego_lane_ID)
            #print(temp_ego_lane_ID.id)
            #if true:
            #    left_lane_info = temp_ego_lane_ID.edgeLeft.ecefEdge
            #    right_lane_info = temp_ego_lane_ID.edgeRight.ecefEdge
            #    edge_left_geo_points =  [ad.map.point.toGeo(p) for p in left_lane_info]
            #    edge_left_enu_points =  [ad.map.point.toENU(p) for p in edge_left_geo_points]
            #    edge_right_geo_points = [ad.map.point.toGeo(p) for p in right_lane_info]
            #    edge_right_enu_points = [ad.map.point.toENU(p) for p in edge_right_geo_points]
            #    #for idx in range(0, size(edge_left_enu_points)):
            #        #print(edge_left_enu_points[idx].x)
            #        #print(edge_left_enu_points[idx].y)
            #        #print("###########")
            #        #print(edge_right_enu_points[idx].x)
            #        #print(edge_right_enu_points[idx].y)
        #