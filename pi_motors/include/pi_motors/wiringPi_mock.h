#ifndef WIRING_PI_MOCK_H
#define WIRING_PI_MOCK_H

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

// Stubs for building tests off target

const int OUTPUT = 123;
const int INPUT = 124;
const int HIGH = 1;
const int LOW = 0;
const int PUD_OFF = 222;
const int PUD_DOWN = 223;
const int PUD_UP = 224;

//wiringPiSetupGpio()
//pinMode(pin, input/output)
//digitalWrite(pin, high/low)
//delay(miliseconds)
//delayMicroseconds(microseconds)
//void pwmWrite(int pin, int value)

int wiringPiSetupGpio()
{
    return 0;
}

void pinMode(int pin, int mode)
{
    //intentionally empty
}

void digitalWrite(int pin, int value)
{
    //intentionally empty
}

void pwmWrite(int pin, int value)
{
    //intentionally empty
}

void pullUpDnControl(int pin, int pud)
{
    //intentionally empty
}

void delay(int milliseconds)
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(milliseconds));
}

void delayMicroseconds(int microseconds)
{
    boost::this_thread::sleep(boost::posix_time::microseconds(microseconds));
}

#endif
