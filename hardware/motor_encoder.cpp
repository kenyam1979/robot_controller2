#include <cstdlib>
#include <pigpio.h>
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

        if (gpioInitialise() < 0)
        {
            std::cerr << "Failed to initialize pigpio" << std::endl;
        }

        velocity_ = 0.0;
        count_ = 0;
        gpioSetMode(pin_, PI_INPUT);
        gpioSetPullUpDown(pin_, PI_PUD_UP);
        gpioSetAlertFuncEx(pin_, _encCallbackEx, this);

        std::cout << "##### Motor Encoder initiated with pin=" << p << " #####" << std::endl;
    }

    int MotorEncoder::getEncodderCount()
    {
        return count_;
    }

    double MotorEncoder::getVelocity(double dt)
    {
        velocity_ = (count_ - prev_count_) / encoder_tooth_ / dt * (2.0 * M_PI * wheel_radius_);
        prev_count_ = count_;

        std::cout << "##### Motor velocity=" << velocity_ << " #####" << std::endl;
        return velocity_;
    }

    MotorEncoder::~MotorEncoder()
    {
        gpioTerminate();
    }

}
