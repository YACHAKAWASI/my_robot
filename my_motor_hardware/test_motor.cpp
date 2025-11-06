#include "my_motor_hardware/bts7960_actuator.hpp"
#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <chrono>
#include <iostream>

int main() {
  my_motor_hardware::SimpleMotor motor;

  motor.on_init(hardware_interface::HardwareInfo{});
  motor.on_activate(rclcpp_lifecycle::State());

  std::cout << "Girando motor..." << std::endl;
  motor.set_velocity(0.8);

  for (int i = 0; i < 50; ++i) {
    motor.write(rclcpp::Time(), rclcpp::Duration::from_seconds(0.1));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cout << "Deteniendo motor..." << std::endl;
  motor.set_velocity(0.0);
  motor.write(rclcpp::Time(), rclcpp::Duration::from_seconds(0.1));

  // Espera un poco para asegurar que el motor se apague
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Desactivar el hardware (apaga los pines del BTS7960)
  motor.on_deactivate(rclcpp_lifecycle::State());

  std::cout << "Motor detenido completamente." << std::endl;
  return 0;
}
