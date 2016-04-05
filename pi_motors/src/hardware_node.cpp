#include <ros/ros.h>
#include <wiringPi.h>


int main(int argv, char **argc)
{
    ros::init(argv, argc, "pi_hw_node");
    ros::spin();
    return 0;
}
