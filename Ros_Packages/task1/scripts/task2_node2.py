#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
import time

def talker():
    servo_angle = rospy.Publisher('servo_angle', Int16, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(20)
    count = 0
    
    #print("Task number '1' published to topic taskNumber")
    while not rospy.is_shutdown():
        for i in range(0,181):
            servo_angle.publish(i)
            rate.sleep()
        for i in range(0,181):
            servo_angle.publish(180-i)
            rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
