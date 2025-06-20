#include <iostream>

#include <pigpiod_if2.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"

#include "robot_controller2/motor.hpp"

namespace motor
{

    Motor::Motor()
    {
    }

    void Motor::initialize(int p1, int p2)
    {

        pin1_ = p1;
        pin2_ = p2;

        pi_ = pigpio_start(NULL, NULL);
        if (pi_ < 0)
        {
            std::cerr << "##### [ERROR] Failed to initialize pigpio" << std::endl;
        }

        set_mode(pi_, pin1_, PI_OUTPUT);
        set_mode(pi_, pin2_, PI_OUTPUT);

        set_PWM_frequency(pi_, pin1_, PWM_FREQ);
        set_PWM_frequency(pi_, pin2_, PWM_FREQ);

        set_PWM_range(pi_, pin1_, MAX_MV);
        set_PWM_range(pi_, pin2_, MAX_MV);

        std::cout << "##### Motor initiated with pin1=" << p1 << " pin2=" << p2 << " #####" << std::endl;
    }

    void Motor::setManipulatingVariable(int mv)
    {

        if (mv > MAX_MV)
            mv = MAX_MV;
        else if (mv < -MAX_MV)
            mv = -MAX_MV;

        if (mv >= 0)
        {
            set_PWM_dutycycle(pi_, pin1_, mv);
            set_PWM_dutycycle(pi_, pin2_, 0);
        }
        else
        {
            set_PWM_dutycycle(pi_, pin1_, 0);
            set_PWM_dutycycle(pi_, pin2_, -mv);
        }
        // Debug
        // std::cout << "##### Motor recieved mv=" << mv << std::endl;
    }

    void Motor::stopMotor()
    {
        set_PWM_dutycycle(pi_, pin1_, 0);
        set_PWM_dutycycle(pi_, pin2_, 0);
        // Debug
        // std::cout << "##### Motor stopped" << std::endl;
    }

    Motor::~Motor()
    {
        pigpio_stop(pi_);
    }
}