#ifndef MOTOR__DIFFBOT_SYSTEM_HPP_
#define MOTOR__DIFFBOT_SYSTEM_HPP_

namespace motor
{
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