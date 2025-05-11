#include <cstdlib>
#include <pigpiod_if2.h>
#include <bits/stdc++.h>
#include <iostream>

#include "robot_controller2/motor_encoder.hpp"

namespace motor_encoder
{

    MotorEncoder::MotorEncoder()
    {
    }
    void MotorEncoder::initialize(int p, double wr, double et)
    {
        pin_ = p;
        wheel_radius_ = wr;
        encoder_tooth_ = et;
        pi_ = pigpio_start(NULL, NULL);
        // if (gpioInitialise() < 0)
        // {
        //     std::cerr << "Failed to initialize pigpio" << std::endl;
        // }

        velocity_ = 0.0;
        angular_velocity_ = 0.0;
        count_ = 0;
        set_mode(pi_, pin_, PI_INPUT);
        set_pull_up_down(pi_, pin_, PI_PUD_UP);
        callback_ex(pi_, pin_, RISING_EDGE, _encCallbackEx, this);

        std::cout << "##### Motor Encoder initiated with pin=" << p << " #####" << std::endl;
    }

    int MotorEncoder::getEncodderCount()
    {
        return count_;
    }

    double MotorEncoder::getVelocity(double dt)
    {
        velocity_ = this->getAngularVelocity(dt) * wheel_radius_;
        prev_count_ = count_;

        // std::cout << "##### Motor velocity=" << velocity_ << " #####" << std::endl;
        return velocity_;
    }

    double MotorEncoder::getAngularVelocity(double dt)
    {
        angular_velocity_ = (count_ - prev_count_) / encoder_tooth_ / dt * (2.0 * M_PI);
        prev_count_ = count_;

        // std::cout << "##### Motor angular velocity=" << angular_velocity_ << " #####" << std::endl;
        return angular_velocity_;
    }

    MotorEncoder::~MotorEncoder()
    {
        pigpio_stop(pi_);
    }

}
