#ifndef MOTOR_ENCODER__DIFFBOT_SYSTEM_HPP_
#define MOTOR_ENCODER__DIFFBOT_SYSTEM_HPP_

namespace motor_encoder
{
    class MotorEncoder
    {

    public:
        MotorEncoder();
        void initialize(int p, double wr, double et);
        int getEncodderCount();
        double getVelocity(double dt);          // m/s
        double getAngularVelocity(double dt);   //rad/s
        ~MotorEncoder();

    private:
        int pi_;
        int pin_;
        double wheel_radius_;
        double encoder_tooth_;

        double velocity_ = 0.0;             // m/s
        double angular_velocity_ = 0.0;     // rad/s
        int count_;
        int prev_count_ = 0;

        void _encCallback(unsigned gpio, unsigned level, uint32_t tick)
        {
            if (level == 1)
            {
                count_ += 1;
                // std::cout << "PIN" << gpio << "=" << count_ << std::endl;
            }
        }

        static void _encCallbackEx(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user)
        {
            MotorEncoder *myself = (MotorEncoder *)user;
            myself->_encCallback(gpio, level, tick);
        }
    };
}

#endif // MOTOR_ENCODER__DIFFBOT_SYSTEM_HPP_