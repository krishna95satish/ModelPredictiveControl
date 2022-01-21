cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Trajectory.srv"
  "msg/ControlAction.msg"
  "msg/States.msg"
  "msg/Trajectories.msg"
  "msg/Trajectory.msg"
  "msg/TrajectoryData.msg"
  "msg/TrajectoryDataPoint.msg"
  "msg/TrajectoryPoint.msg"
  DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs
  diagnostic_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
