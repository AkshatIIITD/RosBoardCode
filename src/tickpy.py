#!/usr/bin/env python

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Int16

# GPIO Pin assignments
left_encoder = 23  # Left encoder
right_encoder = 5  # Right encoder
left_reverse = 22  # Monitors as input that goes low when the left motor is set to reverse
right_reverse = 6  # Monitors as input that goes low when the right motor is set to reverse

# Max and min allowable values
encoder_min = -32768
encoder_max = 32768

left_count = Int16()
right_count = Int16()

def left_event(channel):
    if GPIO.input(left_reverse) == 0:  # Decrement if the motor is commanded to reverse
        if left_count.data == encoder_min:  # Handle rollunder
            left_count.data = encoder_max
        else:
            left_count.data -= 1
    else:  # Increment if not commanded to reverse (must be going forward)
        if left_count.data == encoder_max:  # Handle rollover
            left_count.data = encoder_min
        else:
            left_count.data += 1

def right_event(channel):
    if GPIO.input(right_reverse) == 0:
        if right_count.data == encoder_min:
            right_count.data = encoder_max
        else:
            right_count.data -= 1
    else:
        if right_count.data == encoder_max:
            right_count.data = encoder_min
        else:
            right_count.data += 1

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(left_encoder, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(right_encoder, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(left_reverse, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(right_reverse, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def main():
    setup_gpio()

    rospy.init_node('tick_pub')
    pub_left = rospy.Publisher('left_ticks', Int16, queue_size=1000)
    pub_right = rospy.Publisher('right_ticks', Int16, queue_size=1000)

    GPIO.add_event_detect(left_encoder, GPIO.RISING, callback=left_event)
    GPIO.add_event_detect(right_encoder, GPIO.RISING, callback=right_event)

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        pub_left.publish(left_count)
        pub_right.publish(right_count)
        rate.sleep()

    GPIO.cleanup()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
