#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16

def talker():
    pub = rospy.Publisher('wheel_speed', Int16, queue_size=10)
    ##taskNum = rospy.Publisher('taskNumber', Int16, queue_size=10)
    rospy.init_node('wheel_Spinner', anonymous=True)
    rate = rospy.Rate(0.5)
    
    while not rospy.is_shutdown():
        spin_rate = int(input("Enter spin speed :"))
        pub.publish(spin_rate)
        
        print("Published number ",str(spin_rate)," to ROS topic '/wheel_speed'")
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
