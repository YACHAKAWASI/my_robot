cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

find_package(ament_cmake REQUIRED)

# Solo instalamos los directorios; no necesitamos find_package de launch/launch_ros/xacro
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
