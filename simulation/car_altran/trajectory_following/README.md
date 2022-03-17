# AORTA - Trajectory Following
## Trjectory following package installation
run the following command to install this trajecotry following package,

```
colcon build --packages-select trajectory_following
```

## Launch
After sourcing setup.bash, run the follwong command:

```
ros2 launch trajectory_following trajectory_following.launch.py
```

The Nodes trajectory_following and dummy_vehicle would be launched.

## Nodes
### trajectory_following 
Here PID control is appled in the trajectory following node. According to the deviations of longitudinal and lateral postions, heading angle, and velocity bwtween vehicle status and reference trajectory, desired steering angle and acceleration +/- can be calculated and sent to vehilce for motion control.

### dummy-vehilce
In order to test the trajectory following function, a dummy vehicle model is built here and serves as a ROS2 Node. A simple kinematic bicycle model is defined to present vehicle motion behaviour. 

## Workflow
### input and output of trajectory following
Input
- dummy vehicle node or carla ego vehicle pubulishs ROS topic '/aorta/altran/vehicle' in every cycle to update vehicle status
- trajectory planner pubulishs ROS topic '/aorta/trajectory/altran_internal in defined cycle to update trajectory list

Output(dummy vehicle): 
- trajectory following node publishs ROS topic '/aorta/altran/trajectory_following_dummy' in every cycle

Output(carla ego vehicle): 
- trajectory following node publishs ROS topic '/aorta/altran/trajectory_following' in every cycle

### input and output of dummy vehicle
Input 
- trajectory following node publishs ROS topic '/aorta/altran/trajectory_following_dummy' in every cycle

Output
- dummy vehicle node pubulishs ROS topic '/aorta/altran/vehicle' in every cycle to update vehicle status
