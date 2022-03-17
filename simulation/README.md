# AORTA Simulation Module
## Description
This module contains all the nodes needed for running the AORTA simulation using ROS. <br/>
The module contains 19 ROS nodes in total with the following structure:
```
.
├── akka_filter
│   ├── akka_filter
│   ├── Components
│   ├── launch
│   ├── resource
│   └── test
├── akka_sensor_dummy
│   ├── akka_sensor_dummy
│   ├── Components
│   ├── launch
│   ├── resource
│   └── test
├── altran_filter
│   ├── altran_filter
│   ├── Components
│   ├── launch
│   ├── resource
│   └── test
├── aorta_server
│   ├── aorta_server
│   ├── Components
│   ├── launch
│   ├── resource
│   └── test
├── car_akka
│   ├── car_akka
│   ├── Components
│   ├── launch
│   ├── resource
│   └── test
├── car_altran
|   ├── car_altran
|   │   ├── car_altran
|   │   ├── Components
|   │   ├── launch
|   │   ├── resource
|   │   └── test
|   ├── trajectory_planning
|   ├── trajectory_following
├── car_default
│   ├── car_default
│   ├── Components
│   ├── launch
│   ├── resource
│   └── test
├── car_emergency
│   ├── car_emergency
│   ├── Components
│   ├── launch
│   ├── resource
│   └── test
├── car_tuk
│   ├── car_tuk
│   ├── Components
│   ├── launch
│   ├── resource
│   └── test
├── default_filter
│   ├── Components
│   ├── default_filter
│   ├── launch
│   ├── resource
│   └── test
├── dummy_manual_control
│   ├── Components
│   ├── dummy_manual_control
│   ├── launch
│   ├── resource
│   └── test
├── edge_device
│   ├── Components
│   ├── edge_device
│   ├── launch
│   ├── resource
│   └── test
├── emergency_filter
│   ├── Components
│   ├── emergency_filter
│   ├── launch
│   ├── resource
│   └── test
├── resource
├── ros_bridge_addon
│   ├── Components
│   ├── launch
│   ├── resource
│   ├── ros_bridge_addon
│   └── test
├── traffic_light_control
│   ├── Components
│   ├── launch
│   ├── resource
│   ├── test
│   └── traffic_light_control
├── tuk_filter
|   ├── Components
|   ├── launch
|   ├── resource
|   ├── test
|   └── tuk_filter
└── launch
```
### Filter nodes
For each actor in the simulation there is a filter node that filters the communication from the AORTA server (please refer to the architecture). <br/>

These filters are named <*actor*>_filter, where <*actor*> is one of 5 possible types:
1. altran
2. tuk
3. akka
4. default
5. emergency

The filters can be parameterized with "drop_probability" and "delay_probability" for each of the publisher topics. The names for the parameters must match the name of the publisher e.g. "gnss" in the following example would be a valid configuration:

```
akka_filter_node:
  ros__parameters:
    use_sim_time: false
    topics:
        publisher:
            gnss: "/aorta/car_info/filtered/akka/gnss"
    communication:
        gnss:
            drop_probability: 0.3
            delay_probability: 0.2
            delay_time_ms: 200
```
The filter callback then uses the topic name to reference the corresponding publisher and will either drop and/or delay the messages based on the topic configuration.

### Car nodes
There are also nodes that control the actors in the simulation and are similarily named: car_<*actor*>

### Car Altran
This folder includes the simple simulation similar to the other cars. There is also a possiblility to use a smarter trajectory planning and trajectory following instead.

## Launching the simulation
For the simulation to run correctly, all the actor nodes should be started along with the AORTA server node. 

This should be done only after running the CARLA simulator; the CARLA **ros-bridge** node; and the AORTA test (defined in ros_bridge_addon).

The **launch** folder contains launch files that can be used for launching test scenarios.

To simulate all the vehicles including altran vehicle:
```
ros2 launch src/simulation/launch/simulation.launch.py
```
To simulate the altran vehicle using trajectory planning and following along with other vehicles:
```
ros2 launch src/simulation/launch/simulation_with_tp_tf.launch.py
```

### Behavior of the simulation
When correctly starting the simulation, all the actors will be spawned (using the AORTA test) and after 1 minute the emergency vehicle will commence moving according to a predefined set of waypoints. All other vehicles should move to the locations received from the AORTA_server node.

For a pre-recorded demonstration of this simulation please see this  [link](https://git.altran.de/adas/aorta/uploads/5f7b1d0aa422f3eb355e67e681159e0b/aorta_test_scenario.mkv).


# Simulation

## Dummy Scenario

##### Running the dummy scenario

To run the dummy scenario you have to follow following steps

1. Start CARLA
2. Start carla-ros-bridge
```
ros2 launch carla_aorta_test carla_aorta_test.launch.py
```
3.  Start simulation nodes

##### Explanation
__Cars and their positions:__

All the cars, their positions and sensors are described in the objects_aorta.json file of the carla_aorta_test.
If you want to add an additional car you have to add it there, give it a name and make sure it has at least an odometry, tf and control sensor.

Additionally the car needs to have a carla_ad_agent listening for the controlling commands. These are added in the carla_aorta_test.launch.py of the carla_aorta_test.

__Waypoints:__

The waypoints are sent by the AORTA server (server) every 60 seconds, so it will take some time after starting everything until the cars move. They are published as Trajectory, forwarded by the filter-nodes (akka_filter, altran_filter, tuk_filter, emergency_filter) and then consumed by the cars (car_akka, car_altran, car_tuk, car_emergency).

The cars transform the waypoints (Trajectory) into a Path what is needed as input for the carla-ros-bridge.

__Visualize:__

If you want to see everything in the simulation you have to move the camera from the starting point of the map to the left after starting the ros-bridge. There you can find the cars.
