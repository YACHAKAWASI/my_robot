#ifndef MY_MOTOR_PKG__SIMPLE_MOTOR_HPP_
#define MY_MOTOR_PKG__SIMPLE_MOTOR_HPP_

#include <string>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Forward declarations (para no incluir gpiod.h aquí)
struct gpiod_chip;
struct gpiod_line;

namespace my_motor_hardware
{

class SimpleMotor : public hardware_interface::ActuatorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SimpleMotor)

  SimpleMotor() = default;

  // --- Inicialización y ciclo de vida ---
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // --- Exportar interfaces de estado y comando ---
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // --- Lectura y escritura del hardware ---
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Setter público para pruebas
  void set_velocity(double v) { cmd_velocity_ = v; }

private:
  // Estado y comando
  double velocity_{0.0};
  double cmd_velocity_{0.0};
  bool hw_active_{false};

  // ==== libgpiod ====
  gpiod_chip *chip_{nullptr};
  gpiod_line *rpwm_line_{nullptr};
  gpiod_line *lpwm_line_{nullptr};
  gpiod_line *en_line_{nullptr};

  // Pines GPIO (modo BCM)
  int gpio_rpwm_{5};
  int gpio_lpwm_{6};
  int gpio_en_{12};
};

}  // namespace my_motor_hardware

#endif  // MY_MOTOR_PKG__SIMPLE_MOTOR_HPP_
