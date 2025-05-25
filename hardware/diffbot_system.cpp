#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "robot_controller2/diffbot_system.hpp"
#include "robot_controller2/motor_pid.hpp"
#include "robot_controller2/motor.hpp"
#include "robot_controller2/motor_encoder.hpp"

#define CONTROL_FREQ 0.5 // Hz

#define MOT_L_1 23
#define MOT_L_2 22
#define MOT_R_1 18
#define MOT_R_2 17

#define ENC_L 2
#define ENC_R 10

#define WHEEL_RADIUS 0.035 // Meter
#define ENCODER_TOOTH 40.0 // 40 puls per revolution

namespace robot_controller2
{
  hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    logger_ = std::make_shared<rclcpp::Logger>(
        rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.DiffBot"));
    clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

    hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // DiffBotSystem has exactly two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
            joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
            joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
            joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
            joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
            hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {

    motor_pid_left_.initialize();
    motor_pid_right_.initialize();

    motor_left_.initialize(MOT_L_1, MOT_L_2);
    motor_right_.initialize(MOT_R_1, MOT_R_2);

    motor_encoder_left_.initialize(ENC_L, WHEEL_RADIUS, ENCODER_TOOTH);
    motor_encoder_right_.initialize(ENC_R, WHEEL_RADIUS, ENCODER_TOOTH);

    // Set default values
    for (auto i = 0u; i < hw_positions_.size(); i++)
    {
      if (std::isnan(hw_positions_[i]))
      {
        hw_positions_[i] = 0;
        hw_velocities_[i] = 0;
        hw_commands_[i] = 0;
      }
    }

    RCLCPP_INFO(get_logger(), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {

    motor_left_.stopMotor();
    motor_right_.stopMotor();
    motor_pid_left_.reset();
    motor_pid_right_.reset();

    RCLCPP_INFO(get_logger(), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type DiffBotSystemHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {

    double reverse_flg = 1.0;

    if (hw_commands_[0] < 0)
    {
      reverse_flg = -1.0;
    }
    else
    {
      reverse_flg = 1.0;
    }
    hw_velocities_[0] = motor_encoder_left_.getAngularVelocity(period.seconds()) * reverse_flg;
    hw_positions_[0] = hw_positions_[0] + period.seconds() * hw_velocities_[0] * WHEEL_RADIUS * reverse_flg;

    if (hw_commands_[1] < 0)
    {
      reverse_flg = -1.0;
    }
    else
    {
      reverse_flg = 1.0;
    }
    hw_velocities_[1] = motor_encoder_right_.getAngularVelocity(period.seconds()) * reverse_flg;
    hw_positions_[1] = hw_positions_[1] + period.seconds() * hw_velocities_[1] * WHEEL_RADIUS * reverse_flg;

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type DiffBotSystemHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {

    int mv_left;
    int mv_right;

    mv_left = motor_pid_left_.calculateManipulatingVariable(hw_commands_[0], hw_velocities_[0], period.seconds());
    mv_right = motor_pid_right_.calculateManipulatingVariable(hw_commands_[1], hw_velocities_[1], period.seconds());

    motor_left_.setManipulatingVariable(mv_left);
    motor_right_.setManipulatingVariable(mv_right);

    return hardware_interface::return_type::OK;
  }

} // namespace robot_controller2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    robot_controller2::DiffBotSystemHardware, hardware_interface::SystemInterface)
