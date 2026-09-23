cmake_minimum_required(VERSION 3.8)
project(ssc_class_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  worlds
  models
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/drive_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
