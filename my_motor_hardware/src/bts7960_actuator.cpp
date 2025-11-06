#include "my_motor_hardware/bts7960_actuator.hpp"

#include <gpiod.h>
#include <algorithm>
#include <cmath>
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_motor_hardware
{

// =============== Ciclo de vida ===============

hardware_interface::CallbackReturn SimpleMotor::on_init(
  const hardware_interface::HardwareInfo & /*info*/)
{
  hw_active_ = false;
  velocity_ = 0.0;
  cmd_velocity_ = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SimpleMotor::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Abrir chip (gpiochip0 en Raspberry Pi)
  chip_ = gpiod_chip_open_by_name("gpiochip0");
  if (!chip_) {
    RCLCPP_ERROR(rclcpp::get_logger("my_motor_hardware"), "No se pudo abrir gpiochip0");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Obtener líneas
  rpwm_line_ = gpiod_chip_get_line(chip_, gpio_rpwm_);
  lpwm_line_ = gpiod_chip_get_line(chip_, gpio_lpwm_);
  en_line_   = gpiod_chip_get_line(chip_, gpio_en_);
  if (!rpwm_line_ || !lpwm_line_ || !en_line_) {
    RCLCPP_ERROR(rclcpp::get_logger("my_motor_hardware"), "No se pudieron obtener líneas GPIO");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Solicitar líneas como salida
  if (gpiod_line_request_output(rpwm_line_, "bts7960_rpwm", 0) < 0 ||
      gpiod_line_request_output(lpwm_line_, "bts7960_lpwm", 0) < 0 ||
      gpiod_line_request_output(en_line_,   "bts7960_en",   0) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("my_motor_hardware"), "No se pudieron reservar líneas GPIO");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Estado inicial seguro
  gpiod_line_set_value(rpwm_line_, 0);
  gpiod_line_set_value(lpwm_line_, 0);
  gpiod_line_set_value(en_line_,   0);

  cmd_velocity_ = 0.0;
  velocity_ = 0.0;
  hw_active_ = true;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SimpleMotor::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Llevar a estado seguro y liberar
  if (en_line_)   gpiod_line_set_value(en_line_, 0);
  if (rpwm_line_) gpiod_line_set_value(rpwm_line_, 0);
  if (lpwm_line_) gpiod_line_set_value(lpwm_line_, 0);

  if (rpwm_line_) { gpiod_line_release(rpwm_line_); rpwm_line_ = nullptr; }
  if (lpwm_line_) { gpiod_line_release(lpwm_line_); lpwm_line_ = nullptr; }
  if (en_line_)   { gpiod_line_release(en_line_);   en_line_   = nullptr; }
  if (chip_)      { gpiod_chip_close(chip_);        chip_      = nullptr; }

  cmd_velocity_ = 0.0;
  hw_active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

// =============== Exportación de interfaces ===============

std::vector<hardware_interface::StateInterface> SimpleMotor::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back("simple_motor", hardware_interface::HW_IF_VELOCITY, &velocity_);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SimpleMotor::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back("simple_motor", hardware_interface::HW_IF_VELOCITY, &cmd_velocity_);
  return command_interfaces;
}

// =============== Bucle de lectura/escritura ===============

hardware_interface::return_type SimpleMotor::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Sin sensores: reflejar comando como estado (simulación simple)
  velocity_ = cmd_velocity_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SimpleMotor::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!hw_active_) {
    return hardware_interface::return_type::OK;
  }

  cmd_velocity_ = std::clamp(cmd_velocity_, -1.0, 1.0);

  // Dirección + ON/OFF (sin PWM con libgpiod)
  if (cmd_velocity_ > 0.0) {
    gpiod_line_set_value(lpwm_line_, 0);
    gpiod_line_set_value(rpwm_line_, 1);  // adelante full
  } else if (cmd_velocity_ < 0.0) {
    gpiod_line_set_value(rpwm_line_, 0);
    gpiod_line_set_value(lpwm_line_, 1);  // atrás full
  } else {
    gpiod_line_set_value(rpwm_line_, 0);
    gpiod_line_set_value(lpwm_line_, 0);  // parado
  }

  // Enable
  gpiod_line_set_value(en_line_, (cmd_velocity_ != 0.0) ? 1 : 0);

  return hardware_interface::return_type::OK;
}

}  // namespace my_motor_hardware

// Registrar el plugin
PLUGINLIB_EXPORT_CLASS(my_motor_hardware::SimpleMotor, hardware_interface::ActuatorInterface)
