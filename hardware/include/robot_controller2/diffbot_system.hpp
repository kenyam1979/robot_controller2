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

#ifndef ROBOT_CONTROLLER2__DIFFBOT_SYSTEM_HPP_
#define ROBOT_CONTROLLER2__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
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

#include "robot_controller2/motor_pid.hpp"
#include "robot_controller2/motor.hpp"
#include "robot_controller2/motor_encoder.hpp"

namespace robot_controller2
{
  class DiffBotSystemHardware : public hardware_interface::SystemInterface
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware);

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

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

    // Objects for logging
    std::shared_ptr<rclcpp::Logger> logger_;
    rclcpp::Clock::SharedPtr clock_;

    // Store the command for the simulated robot
    std::vector<double> hw_commands_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;

    motor_pid::MotorPID motor_pid_left_;
    motor_pid::MotorPID motor_pid_right_;
    motor::Motor motor_left_;
    motor::Motor motor_right_;
    motor_encoder::MotorEncoder motor_encoder_left_;
    motor_encoder::MotorEncoder motor_encoder_right_;
  };

} // namespace test_control

#endif // ROBOT_CONTROLLER2__DIFFBOT_SYSTEM_HPP_
