#include "my_motor_hardware/bts7960_actuator.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace my_motor_hardware
{

hardware_interface::CallbackReturn
Bts7960Actuator::on_init(const hardware_interface::HardwareInfo & info)
{
  (void)info;  // por ahora no usamos parámetros
  RCLCPP_INFO(rclcpp::get_logger("Bts7960Actuator"), "Inicializando hardware BTS7960 (plantilla)...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
Bts7960Actuator::export_state_interfaces()
{
  // De momento no exponemos estados (plantilla mínima)
  return {};
}

std::vector<hardware_interface::CommandInterface>
Bts7960Actuator::export_command_interfaces()
{
  // De momento no exponemos comandos (plantilla mínima)
  return {};
}

hardware_interface::CallbackReturn
Bts7960Actuator::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("Bts7960Actuator"), "Activado.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
Bts7960Actuator::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("Bts7960Actuator"), "Desactivado.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

}  // namespace my_motor_hardware

PLUGINLIB_EXPORT_CLASS(my_motor_hardware::Bts7960Actuator, hardware_interface::SystemInterface)
