
import ad_map_access as ad
from numpy import size

if __name__ == '__main__':
    ad.map.access.init("Town05.txt")

    # map matching
    mapMatching = ad.map.match.AdMapMatching()
    #geoPoint = ad.map.point.GeoPoint()
    egoPosePoint = ad.map.point.ENUPoint()

    # GeoPoint(longitude:-0.000904582,latitude:0.000858925,altitude:0)
    #geoPoint.latitude = ad.map.point.Latitude(0.0012591043674774482)   
    #geoPoint.longitude = ad.map.point.Longitude(0.00034971400577713603) 
    #geoPoint.altitude = ad.map.point.Altitude(0.0)

    ############ Ego position in x, y, z ########################
    egoPosePoint.x = ad.map.point.ENUCoordinate(34.0)
    egoPosePoint.y = ad.map.point.ENUCoordinate(120.0)
    egoPosePoint.z = ad.map.point.ENUCoordinate(0.0)

    mapMatchingResults = mapMatching.getMapMatchedPositions(
        egoPosePoint, ad.physics.Distance(0.5), ad.physics.Probability(0.1))

    if len(mapMatchingResults) > 0:
        pos = mapMatchingResults[0]
        for tempLaneID in mapMatchingResults:
            print(tempLaneID)
        lane = ad.map.lane.getLane(mapMatchingResults[0].lanePoint.paraPoint.laneId)
        print(f"laneId: {lane.id} with {len(lane.contactLanes)} contact lanes:")
        leftLaneInfo = lane.edgeRight
        pointECEObj = leftLaneInfo.ecefEdge
        edge_left_ecef_points = lane.edgeLeft.ecefEdge
        edge_right_ecef_points = lane.edgeRight.ecefEdge
       # print(edge_left_ecef_points)
        edge_left_geo_points =  [ad.map.point.toGeo(p) for p in edge_left_ecef_points]
        edge_left_enu_points =  [ad.map.point.toENU(p) for p in edge_left_geo_points]
        edge_right_geo_points = [ad.map.point.toGeo(p) for p in edge_right_ecef_points]
        edge_right_enu_points = [ad.map.point.toENU(p) for p in edge_right_geo_points]
        print("Hello")
        for idx in range(0, size(edge_left_enu_points)):
            print(edge_left_enu_points[idx].x)
            print(edge_left_enu_points[idx].y)
        #print(edge_left_enu_points[1])
