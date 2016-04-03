#!/usr/bin/env python

"""Hardware interface

Interface node for anything using the RPi's GPIOs
"""

import RPi.GPIO as GPIO
import rospy





if __name__ == '__main__':
    GPIO.setmode(GPIO.BCM)
    try:
        rospy.init_node("gpio_interface")
        rospy.spin()
    finally:
        GPIO.cleanup()
