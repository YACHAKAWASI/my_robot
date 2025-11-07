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
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(bts7960_actuator
  ${GPIOD_LIB}
  pthread
  rt
)

ament_target_dependencies(bts7960_actuator
  rclcpp rclcpp_lifecycle hardware_interface pluginlib
)

# Exportar la descripción del plugin
pluginlib_export_plugin_description_file(hardware_interface my_motor_hardware.xml)

# ===== Ejecutable de prueba (opcional) =====
add_executable(test_motor
  test_motor.cpp
)

target_include_directories(test_motor PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
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
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(TARGETS test_motor
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(FILES my_motor_hardware.xml DESTINATION share/${PROJECT_NAME})

# ===== Exports =====
ament_export_libraries(bts7960_actuator)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_lifecycle hardware_interface pluginlib)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
