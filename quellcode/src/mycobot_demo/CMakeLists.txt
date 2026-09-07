cmake_minimum_required(VERSION 3.8)
project(mycobot_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pick_place_cartesian src/pick_place_cartesian.cpp)
ament_target_dependencies(pick_place_cartesian
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  tf2_geometry_msgs
  control_msgs
  trajectory_msgs
)

add_executable(approach_target src/approach_target.cpp)
ament_target_dependencies(approach_target
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  tf2_geometry_msgs
  visualization_msgs
  control_msgs
  trajectory_msgs
)

add_executable(pick_tilt src/pick_tilt.cpp)
ament_target_dependencies(pick_tilt
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  tf2_geometry_msgs
  visualization_msgs
  control_msgs
  trajectory_msgs
  std_msgs
)

add_executable(goto_clicked_point src/goto_clicked_point.cpp)
ament_target_dependencies(goto_clicked_point
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  tf2_geometry_msgs
  visualization_msgs
)

install(TARGETS pick_place_cartesian approach_target pick_tilt goto_clicked_point
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
