#ifndef MOTOR_ENCODER__DIFFBOT_SYSTEM_HPP_
#define MOTOR_ENCODER__DIFFBOT_SYSTEM_HPP_

namespace motor_encoder
{
    class MotorEncoder
    {

    public:
        MotorEncoder(int p, double wr, double et);
        void initialize(int p, double wr, double et);
        int getEncodderCount();
        double getVelocity(double dt);
        ~MotorEncoder();

    private:
        int pin_;
        double wheel_radius_;
        double encoder_tooth_;

        double velocity_ = 0.0;
        int count_;
        int prev_count_ = 0;

        void _encCallback(int gpio, int level, uint32_t tick)
        {
            if (level == 1)
            {
                count_ += 1;
                // std::cout << "PIN" << gpio << "=" << count_ << std::endl;
            }
        }

        static void _encCallbackEx(int gpio, int level, uint32_t tick, void *user)
        {
            MotorEncoder *myself = (MotorEncoder *)user;
            myself->_encCallback(gpio, level, tick);
        }
    };
}

#endif // MOTOR_ENCODER__DIFFBOT_SYSTEM_HPP_