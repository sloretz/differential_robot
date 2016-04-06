#include <ros/ros.h>
#include "pi_motors/wiringPi_testhook.h"
#include "pi_motors/28byj48.hpp"

int main(int argv, char **argc)
{
    ros::init(argv, argc, "pi_hw_node");
    wiringPiSetupGpio();

    //BCM pin numbers
    pi_motors::Stepper28byj48 s1(17, 27, 22, 23);
    s1.start();
    s1.set_speed(1);
    ros::spin();
    return 0;
}
