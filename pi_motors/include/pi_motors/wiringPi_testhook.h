#ifndef WIRING_PI_TESTHOOK_H
#define WIRING_PI_TESTHOOK_H

#ifdef __arm__
#include <wiringPi.h>
#else //not arm
#include "pi_motors/wiringPi_mock.h"
#endif
#endif
