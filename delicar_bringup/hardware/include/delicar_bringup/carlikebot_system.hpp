#ifndef DELICAR_BRINGUP__CARLIKEBOT_SYSTEM_HPP_
#define DELICAR_BRINGUP__CARLIKEBOT_SYSTEM_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "delicar_bringup/actuator.hpp"
#include "delicar_bringup/delicar_comms.hpp"

namespace delicar_bringup
{

class CarlikeBotSystemHardware : public hardware_interface::SystemInterface
{

struct Config
{
  std::string drive_wheel_name = "virtual_rear_wheel_joint";
  std::string steer_wheel_name = "virtual_front_wheel_joint";
  std::string serial_port = "/dev/ttyUSB0";
  int baud_rate = 9600;
  char parity = 'O';
  int data_bits = 8;
  int stop_bits = 1;
  int slave_id = 1;
  int traction_speed_cmd_register = 0;
  int traction_speed_fb_register = 8;
  int steering_angle_cmd_register = 20;
  int steering_angle_fb_register = 22;
  double steering_scale_factor = 1.0;
  double traction_scale_factor = 1.0;
};

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CarlikeBotSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Get the logger of the SystemInterface.
  /**
   * \return logger of the SystemInterface.
   */
  rclcpp::Logger get_logger() const { return *logger_; }

  /// Get the clock of the SystemInterface.
  /**
   * \return clock of the SystemInterface.
   */
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

private:
  DelicarComms comms_;

  Actuator drive_actuator_;
  Actuator steer_actuator_;

  Config hw_params_;

  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace delicar_bringup

#endif  // DELICAR_BRINGUP__CARLIKEBOT_SYSTEM_HPP_
