cmake_minimum_required(VERSION 3.8)
project(my_motor_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_index_cpp REQUIRED)  # <-- añadido

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/bts7960_actuator.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  ament_index_cpp                  # <-- añadido
)

# Registra el descriptor del plugin en ament index
pluginlib_export_plugin_description_file(hardware_interface my_motor_hardware.xml)

# Instalación
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES my_motor_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export (buenas prácticas)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp hardware_interface pluginlib ament_index_cpp)

ament_package()
