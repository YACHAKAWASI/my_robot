cmake_minimum_required(VERSION 3.8)
project(my_motor_hardware)

# ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# libgpiod
find_library(GPIOD_LIB gpiod REQUIRED)

# ===== Librería del hardware (plugin) =====
add_library(bts7960_actuator SHARED
  src/bts7960_actuator.cpp
)
target_include_directories(bts7960_actuator PUBLIC
  include
)
target_link_libraries(bts7960_actuator
  ${GPIOD_LIB}
  pthread
  rt
)
ament_target_dependencies(bts7960_actuator
  rclcpp rclcpp_lifecycle hardware_interface pluginlib
)

# Exportar el plugin (pluginlib)
pluginlib_export_plugin_description_file(hardware_interface my_motor_hardware.xml)

# ===== Ejecutable de prueba =====
add_executable(test_motor
  test_motor.cpp
)
target_include_directories(test_motor PRIVATE
  include
)
target_link_libraries(test_motor
  bts7960_actuator
  ${GPIOD_LIB}
  pthread
  rt
)
ament_target_dependencies(test_motor
  rclcpp rclcpp_lifecycle hardware_interface
)

# ===== Instalación =====
install(TARGETS bts7960_actuator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS test_motor
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(FILES my_motor_hardware.xml DESTINATION share/${PROJECT_NAME})

ament_package()
