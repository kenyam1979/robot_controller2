#ifndef MOTOR__DIFFBOT_SYSTEM_HPP_
#define MOTOR__DIFFBOT_SYSTEM_HPP_

namespace motor
{

    #define MAX_MV 500.0
    #define PWM_FREQ 60

    class Motor
    {
    public:
        Motor();
        void initialize(int p1, int p2);
        void setManipulatingVariable(int mv);
        void stopMotor();
        ~Motor();

    private:
        int pi_;
        int pin1_;
        int pin2_;
    };
}

#endif // MOTOR__DIFFBOT_SYSTEM_HPP_