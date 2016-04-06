#ifndef STEPPER_28BYJ48_HPP
#define STEPPER_28BYJ48_HPP

#include <boost/thread.hpp>

namespace pi_motors
{
    class Stepper28byj48
    {
        public:
            Stepper28byj48(int pin0, int pin1, int pin2, int pin3);
            ~Stepper28byj48();

            void start();
            void stop();
            void set_speed(double radians_per_second);

        protected:
            int pins_[4];
            double cur_speed_;
            double next_speed_;
            double accel_;
            bool stopping_;
            boost::thread update_thread_;

            void run();
    };
}

#endif
