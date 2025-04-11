#include "delicar_bringup/carlikebot_system.hpp"
#include "delicar_bringup/actuator.hpp"
#include "delicar_bringup/delicar_comms.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace delicar_bringup
{

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.CarlikeBot"));

  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  // Check if the number of joints is correct based on the mode of operation
  if (info_.joints.size() != 2)
  {
    RCLCPP_ERROR(
      get_logger(),
      "CarlikeBotSystemHardware::on_init() - Failed to initialize, "
      "because the number of joints %ld is not 2.",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get parameters from URDF
  hw_params_.drive_wheel_name = info_.hardware_parameters["drive_wheel_name"];
  hw_params_.steer_wheel_name = info_.hardware_parameters["steer_wheel_name"];
  hw_params_.serial_port = info_.hardware_parameters["serial_port"];
  hw_params_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  hw_params_.parity = info_.hardware_parameters["parity"][0];
  hw_params_.data_bits = std::stoi(info_.hardware_parameters["data_bits"]);
  hw_params_.stop_bits = std::stoi(info_.hardware_parameters["stop_bits"]);
  hw_params_.slave_id = std::stoi(info_.hardware_parameters["slave_id"]);
  
  // Control and feedback registers
  hw_params_.traction_speed_cmd_register = std::stoi(info_.hardware_parameters["traction_speed_cmd_register"]);
  hw_params_.traction_speed_fb_register = std::stoi(info_.hardware_parameters["traction_speed_fb_register"]);
  hw_params_.steering_angle_cmd_register = std::stoi(info_.hardware_parameters["steering_angle_cmd_register"]);
  hw_params_.steering_angle_fb_register = std::stoi(info_.hardware_parameters["steering_angle_fb_register"]);
  
  // Scaling factors
  hw_params_.steering_scale_factor = std::stod(info_.hardware_parameters["steering_scale_factor"]);
  hw_params_.traction_scale_factor = std::stod(info_.hardware_parameters["traction_scale_factor"]);
  
  // Setup drive actuator
  drive_actuator_.setup(hw_params_.drive_wheel_name, "velocity");
  
  // Setup steering actuator
  steer_actuator_.setup(hw_params_.steer_wheel_name, "position");

  RCLCPP_INFO(get_logger(), "Hardware parameters loaded");
  RCLCPP_INFO(get_logger(), "Serial port: %s", hw_params_.serial_port.c_str());
  RCLCPP_INFO(get_logger(), "Baud rate: %d", hw_params_.baud_rate);
  RCLCPP_INFO(get_logger(), "Slave ID: %d", hw_params_.slave_id);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CarlikeBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Add traction joint velocity state interface
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    drive_actuator_.name, hardware_interface::HW_IF_VELOCITY, &drive_actuator_.vel));
      
  // Add traction joint position state interface
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      drive_actuator_.name, hardware_interface::HW_IF_POSITION, &drive_actuator_.pos));

  // Add steering joint position state interface
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      steer_actuator_.name, hardware_interface::HW_IF_POSITION, &steer_actuator_.pos));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
CarlikeBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Add traction joint velocity command interface
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    drive_actuator_.name, hardware_interface::HW_IF_VELOCITY, &drive_actuator_.cmd));

  // Add steering joint position command interface
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      steer_actuator_.name, hardware_interface::HW_IF_POSITION, &steer_actuator_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring DelicarHardware");

  // Setup Modbus communication
  comms_.init(
      hw_params_.serial_port, 
      hw_params_.baud_rate, 
      hw_params_.slave_id, 
      hw_params_.parity,
      hw_params_.data_bits, 
      hw_params_.stop_bits
      );
      
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(get_logger(), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating DelicarHardware");
    
  // Connect to the PLC
  if (!comms_.connect()) {
      RCLCPP_ERROR(get_logger(), "Failed to connect to the PLC");
      return hardware_interface::CallbackReturn::ERROR;
  }
  
  // Stop motors at startup
  int zero_value_traction = 0;
  int zero_value_steer = 1000;
  comms_.setMotorValues(zero_value_traction, hw_params_.traction_speed_cmd_register);
  comms_.setMotorValues(zero_value_steer, hw_params_.steering_angle_cmd_register);

  // Initialize command and state values
  drive_actuator_.cmd = 0.0;
  drive_actuator_.vel = 0.0;
  drive_actuator_.pos = 0.0;
  steer_actuator_.cmd = 0.0;
  steer_actuator_.pos = 0.0;
  
  RCLCPP_INFO(get_logger(), "DelicarHardware successfully activated");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating DelicarHardware");
    
  // Stop motors before disconnecting
  int zero_value_traction = 0;
  int zero_value_steer = 1000;
  comms_.setMotorValues(zero_value_traction, hw_params_.traction_speed_cmd_register);
  comms_.setMotorValues(zero_value_steer, hw_params_.steering_angle_cmd_register);
  
  RCLCPP_INFO(get_logger(), "DelicarHardware successfully deactivated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CarlikeBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected()) {
    RCLCPP_ERROR(get_logger(), "Cannot read from disconnected hardware");
    return hardware_interface::return_type::ERROR;
  }

  // Read traction speed feedback
  int traction_fb_raw;
  comms_.readFeedbackValues(traction_fb_raw, hw_params_.traction_speed_fb_register);

  // Convert raw feedback
    if (traction_fb_raw >= 500) {
      drive_actuator_.vel = (traction_fb_raw - 500) / 15.0;
  } else if (traction_fb_raw <= -500) {
      drive_actuator_.vel = (traction_fb_raw + 500) / 40.0;
  } else if (traction_fb_raw > -500 && traction_fb_raw < 500)
  {
      drive_actuator_.vel = 0;
  }

  // Update position
  drive_actuator_.pos += drive_actuator_.vel * period.seconds();

  // Read steering angle feedback
  int steering_fb_raw;
  comms_.readFeedbackValues(steering_fb_raw, hw_params_.steering_angle_fb_register);

  // Convert raw feedback 
  steer_actuator_.pos = static_cast<double>(steering_fb_raw) / hw_params_.steering_scale_factor;

  std::stringstream ss;
  ss << std::fixed << std::setprecision(5);
  
  ss << "Reading states:" << std::endl
     << "\t"
     << "Period: " << period.seconds() << " s" << std::endl
     << "\t"
     << "For '" << drive_actuator_.name << "'"
     << " got feedback: " << traction_fb_raw
     << " convert to drive velocity: " << drive_actuator_.vel 
     << ", position: " << drive_actuator_.pos << std::endl
     << "\t"
     << "For '" << steer_actuator_.name << "'"
     << " got feedback: " << steering_fb_raw 
     << " convert to steer position: " << steer_actuator_.pos << std::endl;
  
  // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type delicar_bringup ::CarlikeBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected()) {
    RCLCPP_ERROR(get_logger(), "Cannot write to disconnected hardware");
    return hardware_interface::return_type::ERROR;
  }
  
  comms_.setMotorValues(10, 8);

  // Convert velocity command to raw value 
  int traction_cmd_raw;

  if (drive_actuator_.cmd > 0)
    traction_cmd_raw = static_cast<int>(drive_actuator_.cmd * 15 + 500);
  else if (drive_actuator_.cmd < 0)
    traction_cmd_raw = static_cast<int>(drive_actuator_.cmd * 40 - 500);
  else
    traction_cmd_raw = 0;

  comms_.setMotorValues(traction_cmd_raw, hw_params_.traction_speed_cmd_register);

  // Convert position command to raw value 
  
  int steering_cmd_raw;

  if (drive_actuator_.cmd <= 0)
    steering_cmd_raw = static_cast<int>(1000 + steer_actuator_.cmd * hw_params_.steering_scale_factor);
  else
    steering_cmd_raw = static_cast<int>(1000 - steer_actuator_.cmd * hw_params_.steering_scale_factor);
  
  if (steering_cmd_raw > 1100) 
  {
    steering_cmd_raw = 1100;
  } 
  else if (steering_cmd_raw < 900) 
  {
    steering_cmd_raw = 900;
  }

  if (std::isnan(steer_actuator_.cmd)) 
  {
    steering_cmd_raw = 1000;
  } 

  comms_.setMotorValues(steering_cmd_raw, hw_params_.steering_angle_cmd_register);

  std::stringstream ss;
  ss << std::fixed << std::setprecision(5);
  
  ss << "Sending commands:" << std::endl
     << "\t"
     << "For '" << drive_actuator_.name << "'"
     << " got drive command: " << drive_actuator_.cmd 
     << " send to PLC: " << traction_cmd_raw << std::endl
  
     << "\t"
     << "For '" << steer_actuator_.name << "'"
     << " got steer command: " << steer_actuator_.cmd 
     << " send to PLC: " << steering_cmd_raw << std::endl;
  
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());

  return hardware_interface::return_type::OK;
}

}  // namespace delicar_bringup

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  delicar_bringup::CarlikeBotSystemHardware, hardware_interface::SystemInterface)
