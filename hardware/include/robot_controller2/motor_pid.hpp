#ifndef MOTOR_PID__DIFFBOT_SYSTEM_HPP_
#define MOTOR_PID__DIFFBOT_SYSTEM_HPP_

namespace motor_pid
{

    class MotorPID
    {
    public:
        MotorPID();
        void initialize();
        int calculateManipulatingVariable(double target_v, double current_v, double dt);
        void reset();
        ~MotorPID();

    private:
        const double kKp = 1.0; // 80.0
        const double kKi = 1.0; //100.0
        const double kKd = 0.05; // 0.1
        const double kMvCoef = 70.0;

        double target_velocity_ = 0.0;
        double current_velocity_ = 0.0;
        int mv_ = 0;

        double error_P_ = 0.0;
        double error_P_prev_ = 0.0;
        double error_I_ = 0.0;
        double error_D_ = 0.0;
    };
}

#endif // MOTOR_PID__DIFFBOT_SYSTEM_HPP_