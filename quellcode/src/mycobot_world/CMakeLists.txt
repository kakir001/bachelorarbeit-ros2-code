cmake_minimum_required(VERSION 3.5)
project(mycobot_world)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
