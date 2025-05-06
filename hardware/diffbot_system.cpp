// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


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



namespace test_control
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

    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    hw_start_sec_ =
        hardware_interface::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
    hw_stop_sec_ =
        hardware_interface::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
    // END: This part here is for exemplary purposes - Please do not copy to your production code
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
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(get_logger(), "Activating ...please wait...");

    // for (auto i = 0; i < hw_start_sec_; i++)
    // {
    //   rclcpp::sleep_for(std::chrono::seconds(1));
    //   RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
    // }
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    motor_pid_left_.initialize();
    motor_pid_right_.initialize();
    
    motor_left_.initialize(MOT_L_1, MOT_L_2);
    motor_right_.initialize(MOT_R_1, MOT_R_2);

    motor_encoder_left_.initialize(ENC_L, WHEEL_RADIUS, ENCODER_TOOTH);
    motor_encoder_right_.initialize(ENC_R, WHEEL_RADIUS, ENCODER_TOOTH);


    // set some default values
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
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

    // for (auto i = 0; i < hw_stop_sec_; i++)
    // {
    //   rclcpp::sleep_for(std::chrono::seconds(1));
    //   RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_stop_sec_ - i);
    // }
    // END: This part here is for exemplary purposes - Please do not copy to your production code

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
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    // std::stringstream ss;
    // ss << "\n Reading states:---------------------------";
    // for (std::size_t i = 0; i < hw_velocities_.size(); i++)
    // {
    //   // Simulate DiffBot wheels's movement as a first-order system
    //   // Update the joint status: this is a revolute joint without any limit.
    //   // Simply integrates
    //   hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];

    //   ss << std::fixed << std::setprecision(2) << std::endl
    //      << "\t"
    //         "position "
    //      << hw_positions_[i] << " and velocity " << hw_velocities_[i] << " for '"
    //      << info_.joints[i].name.c_str() << "'!";
    // }
    // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());

    hw_velocities_[0] = motor_encoder_left_.getVelocity(period.seconds());
    hw_velocities_[1] = motor_encoder_right_.getVelocity(period.seconds());

    hw_positions_[0] = hw_positions_[0] + period.seconds() * hw_velocities_[0];
    hw_positions_[1] = hw_positions_[1] + period.seconds() * hw_velocities_[1];

    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type test_control::DiffBotSystemHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    // std::stringstream ss;
    // ss << "\n Writing commands:---------------------------";
    // for (auto i = 0u; i < hw_commands_.size(); i++)
    // {
    //   // Simulate sending commands to the hardware
    //   hw_velocities_[i] = hw_commands_[i];

    //   ss << std::fixed << std::setprecision(2) << std::endl
    //      << "\t" << "command " << hw_commands_[i] << " for '" << info_.joints[i].name.c_str() << i << "'!";
    // }
    // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());

    int mv_left;
    int mv_right;   

    mv_left = motor_pid_left_.calculateManipulatingVariable(hw_commands_[0], hw_velocities_[0], period.seconds());
    mv_right = motor_pid_right_.calculateManipulatingVariable(hw_commands_[1], hw_velocities_[1], period.seconds());

    motor_left_.setManipulatingVariable(mv_left);
    motor_right_.setManipulatingVariable(mv_right);

    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::return_type::OK;
  }

} // namespace test_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    test_control::DiffBotSystemHardware, hardware_interface::SystemInterface)

/*

ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped "
twist:
linear:
  x: 0.7
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.0"

  */