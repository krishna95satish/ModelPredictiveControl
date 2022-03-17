cmake_minimum_required(VERSION 3.5)
project(custom_messages)

find_package(ros_environment REQUIRED)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

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


rosidl_generate_interfaces(
    ${PROJECT_NAME}
    "msg/BoundingBox.msg"
    "msg/TrajectoryPoint.msg"
    "msg/Trajectory.msg"
    "msg/Trajectories.msg"
    "msg/Object.msg"
    "msg/ObjectList.msg"
    "msg/TrafficLightControl.msg"
    "msg/TrafficLightProgram.msg"
    "msg/AkkaSensor.msg"
    "msg/TrajectoryDataPoint.msg"
    "msg/TrajectoryData.msg"
    "msg/VehicleControlParameter.msg"
    "srv/CarlaTransform.srv"
    DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
    diagnostic_msgs
    ADD_LINTER_TESTS)
  
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
