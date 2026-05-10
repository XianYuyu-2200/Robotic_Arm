#include "gluon_control/gluon_hw_interface.h"
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG_TO_RAD(x) ((x)*M_PI / 180.0)
#define RAD_TO_DEG(x) ((x)*180.0 / M_PI)
#define STEERING_GEAR_RATIO 36
#define RAD_TO_POS(x) ((x / (2 * M_PI)) * STEERING_GEAR_RATIO)
#define POS_TO_RAD(x) ((x * (2 * M_PI)) / STEERING_GEAR_RATIO)

namespace gluon_control
{
hardware_interface::CallbackReturn GluonSystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Verify interfaces
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("GluonSystemInterface"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("GluonSystemInterface"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("GluonSystemInterface"),
        "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("GluonSystemInterface"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GluonSystemInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("GluonSystemInterface"), "Configuring ...please wait...");

  pController_ = ActuatorController::initController();
  Actuator::ErrorsDefine ec;
  uIDArray_ = pController_->lookupActuators(ec);

  if (uIDArray_.size() == 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("GluonSystemInterface"), "No actuators found! Error code: %x", ec);
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("GluonSystemInterface"), "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GluonSystemInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("GluonSystemInterface"), "Activating ...please wait...");

  for(size_t k = 0; k < uIDArray_.size(); k++) {
      ActuatorController::UnifiedID actuator = uIDArray_.at(k);
      RCLCPP_INFO(rclcpp::get_logger("GluonSystemInterface"), "Enabling actuator ID %d", actuator.actuatorID);
      pController_->enableActuator(actuator.actuatorID, actuator.ipAddress);
      pController_->activateActuatorMode(actuator.actuatorID, Actuator::Mode_Profile_Pos);
      pController_->setPosition(actuator.actuatorID, 0);
  }

  // Set initial commands to current positions (0 for now)
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    hw_commands_[i] = 0.0;
    hw_positions_[i] = 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger("GluonSystemInterface"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GluonSystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("GluonSystemInterface"), "Deactivating ...please wait...");
  // pController_->disableAllActuators(); 
  RCLCPP_INFO(rclcpp::get_logger("GluonSystemInterface"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type GluonSystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Implement read logic if the SDK supports reading position
  for (std::size_t i = 0; i < hw_positions_.size(); i++)
  {
    hw_positions_[i] = hw_commands_[i]; 
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GluonSystemInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (std::size_t i = 0; i < hw_commands_.size(); i++)
  {
    if (i < uIDArray_.size()) {
        ActuatorController::UnifiedID actuator = uIDArray_.at(i);
        pController_->setPosition(actuator.actuatorID, RAD_TO_POS(hw_commands_[i]));
    }
  }
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> GluonSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> GluonSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

}  // namespace gluon_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  gluon_control::GluonSystemInterface, hardware_interface::SystemInterface)
