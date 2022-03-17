cmake_minimum_required(VERSION 3.5)

project(trajectory_visualization_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(custom_messages REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)

include_directories(include)

set(TRAJ_VIS_LIB_SRC
  src/trajectory_display.cpp
  src/trajectories_display.cpp
  )

set(TRAJ_VIS_LIB_HEADERS
  include/trajectory_display.h
  include/trajectories_display.h
  )

add_library(${PROJECT_NAME} SHARED
  ${TRAJ_VIS_LIB_HEADERS}
  ${TRAJ_VIS_LIB_SRC}
)

# https://answers.ros.org/question/265610/undefined-reference-to-vtable-for-myviz/
qt5_wrap_cpp(PROCESSED_MOCS
  ${TRAJ_VIS_LIB_HEADERS}
  TARGET ${PROJECT_NAME}
  OPTIONS --no-notes) # Don't display a note for the headers which don't produce a moc_*.cpp

target_sources(${PROJECT_NAME} PRIVATE ${PROCESSED_MOCS}) # This adds generated moc cpps to target

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

target_link_libraries(${PROJECT_NAME}
  rviz_common::rviz_common
  Qt5::Widgets
  )

ament_target_dependencies(${PROJECT_NAME} "rviz_common" "rviz_rendering" "rviz_default_plugins" "custom_messages")

# Settings to improve the build as suggested on https://github.com/ros2/rviz/blob/ros2/docs/plugin_development.md
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)


if(BUILD_TESTING)
  # run linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
