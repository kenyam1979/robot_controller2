#include "robot_controller2/motor.hpp"
#include <iostream>
#include <pigpio.h>

namespace motor
{

    Motor::Motor()
    {
    }

    void Motor::initialize(int p1, int p2)
    {

        pin1_ = p1;
        pin2_ = p2;

        if (gpioInitialise() < 0)
        {
            std::cerr << "Failed to initialize pigpio" << std::endl;
        }
        gpioSetMode(pin1_, PI_OUTPUT);
        gpioSetMode(pin2_, PI_OUTPUT);

        gpioSetPWMfrequency(pin1_, 60);
        gpioSetPWMfrequency(pin2_, 60);

        gpioSetPWMrange(pin1_, 100);
        gpioSetPWMrange(pin2_, 100);

        std::cout << "##### Motor initiated with pin1=" << p1 << " pin2=" << p2 << " #####" << std::endl;
    }

    void Motor::setManipulatingVariable(int mv)
    {
        if (mv > 0)
        {
            gpioPWM(pin1_, mv);
            gpioPWM(pin2_, 0);
        }
        else
        {
            gpioPWM(pin1_, 0);
            gpioPWM(pin2_, -mv);
        }
        std::cout << "##### Motor recieved mv=" << mv << " #####" << std::endl;
    }

    void Motor::stopMotor()
    {
        gpioPWM(pin1_, 0);
        gpioPWM(pin2_, 0);
        std::cout << "##### Motor stopped #####" << std::endl;
    }

    Motor::~Motor()
    {
        gpioTerminate();
    }
}