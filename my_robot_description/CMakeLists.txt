cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

find_package(ament_cmake REQUIRED)

# Instalamos los URDF/Xacro
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
