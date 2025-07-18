#include <iostream>
#include "robot_controller2/motor_pid.hpp"
#include "robot_controller2/motor.hpp"

namespace motor_pid
{


    MotorPID::MotorPID()
    {
    }

    void MotorPID::initialize()
    {
    }

    int MotorPID::calculateManipulatingVariable(double target_v, double current_v, double dt)
    {

        target_velocity_ = target_v;
        current_velocity_ = current_v;

    
        if (target_velocity_ == 0.0) {
            this->reset();
            return 0;
        }

        if (dt <= 0.0)
        {
            std::cout << "##### [WARNING] dt is not valid" << std::endl;
            return mv_;
        }

        error_P_ = target_velocity_ - current_velocity_;
        error_I_ = error_I_ + error_P_ * dt;
        error_D_ = (error_P_ - error_P_prev_) / dt;

        mv_ = (int)((kKp * error_P_ + kKi * error_I_ + kKd * error_D_) * kMvCoef);

        if (mv_ > MAX_MV)
            mv_ = MAX_MV;

        if (mv_ < -MAX_MV)
            mv_ = -MAX_MV;

        error_P_prev_ = error_P_;

        // Debug
        // std::cout << "##### Set volocity to " << target_velocity_ << ", current velocity is " << current_velocity_ << std::endl;
        // std::cout << "##### error_P_=" << error_P_ << " error_I_=" << error_I_ << " error_D_=" << error_D_ << std::endl;

        return mv_;
    }

    void MotorPID::reset()
    {
        target_velocity_ = 0.0;
        current_velocity_ = 0.0;
        mv_ = 0;

        error_P_ = 0.0;
        error_P_prev_ = 0.0;
        error_I_ = 0.0;
        error_D_ = 0.0;

    }

    MotorPID::~MotorPID()
    {
        this->reset();
    }

}