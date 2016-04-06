#include <cmath>
#include "pi_motors/28byj48.hpp"
#include "pi_motors/wiringPi_testhook.h"

namespace pi_motors
{
    Stepper28byj48::Stepper28byj48(int pin0, int pin1, int pin2, int pin3)
    {
        //initialize stepper given pins
        pins_[0] = pin0;
        pins_[1] = pin1;
        pins_[2] = pin2;
        pins_[3] = pin3;
        for (int i = 0; i < 4; i++)
        {
            pinMode(pins_[i], OUTPUT);
        }
        accel_ = 0.1;
    }

    Stepper28byj48::~Stepper28byj48()
    {
        //make sure thread stops
        stopping_ = true;
    }

    void Stepper28byj48::start()
    {
        //create thread and start it. Set initial speed to zero
        cur_speed_ = 0;
        next_speed_ = 0;
        stopping_ = false;
        update_thread_ = boost::thread(&Stepper28byj48::run, this);
    }

    void Stepper28byj48::stop()
    {
        stopping_ = true;
        update_thread_.join();
    }

    void Stepper28byj48::set_speed(double radians_per_second)
    {
        //Set the next target speed, update thread will make it happen
        next_speed_ = radians_per_second;
    }

    void Stepper28byj48::run()
    {
        //Send step pulses and delay as necessary
        //Gradually increase current speed based on accelleration parameters
        /*
         * Dual phase full stepping
         * 1/64 gear reduction (about, depends on manufacturer)
         * 32 steps per revolution
         * 2048 steps per wheel revolution (or 2038 if gear reduction is not exact)
         * 2*pi radians in a revolution
         */
        int sequence = 0;
        while (!stopping_)
        {
            //update cur speed as needed
            if (cur_speed_ != next_speed_)
            {
                double sign = next_speed_ - cur_speed_ > 0 ? 1.0 : -1.0;
                cur_speed_ += accel_ * sign;
                if (std::fabs(cur_speed_) > std::fabs(next_speed_))
                {
                    cur_speed_ = next_speed_;
                }
            }

            //next pulse to motor
            switch (sequence)
            {
                case 0:
                    digitalWrite(pins_[0], HIGH);
                    digitalWrite(pins_[1], HIGH);
                    digitalWrite(pins_[2], LOW);
                    digitalWrite(pins_[3], LOW);
                    break;
                case 1:
                    digitalWrite(pins_[0], LOW);
                    digitalWrite(pins_[1], HIGH);
                    digitalWrite(pins_[2], HIGH);
                    digitalWrite(pins_[3], LOW);
                    break;
                case 2:
                    digitalWrite(pins_[0], LOW);
                    digitalWrite(pins_[1], LOW);
                    digitalWrite(pins_[2], HIGH);
                    digitalWrite(pins_[3], HIGH);
                    break;
                case 3:
                    digitalWrite(pins_[0], LOW);
                    digitalWrite(pins_[1], HIGH);
                    digitalWrite(pins_[2], LOW);
                    digitalWrite(pins_[3], HIGH);
                    break;
                default:
                    break;
            }
            sequence = sequence >= 4 ? 0 : sequence + 1;

            //delay until next pulse time
            const double min_speed = 0.01;
            if (std::fabs(cur_speed_) > 0.01)
            {
                const double steps_per_revolution = 2048;
                const double radians_per_revolution = 2.0 * M_PI;
                const double microseconds_per_second = 1000000;
                const double convertion = microseconds_per_second * radians_per_revolution / steps_per_revolution;
                int microseconds_per_step = convertion / cur_speed_;
                delayMicroseconds(microseconds_per_step);
            }
            else
            {
                const double stationary_delay_miliseconds = 1;
                delay(stationary_delay_miliseconds);
            }
        }
    }

}
