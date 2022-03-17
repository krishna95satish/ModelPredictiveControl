import pymap3d as pm


    
point_latitude= 6378100
point_longitude= 17.8
point_altitude=2.0752711296081543


origin_latitude=0.0
origin_longitude=0.0
origin_altitude=0.0000

carla_x, carla_y, carla_z = pm.geodetic2enu(point_latitude, point_longitude, point_altitude, origin_latitude, origin_longitude, origin_altitude, ell=pm.utils.Ellipsoid('wgs84'))
print ("The converted values from Geodetic to carla are", carla_x, -carla_y, carla_z)