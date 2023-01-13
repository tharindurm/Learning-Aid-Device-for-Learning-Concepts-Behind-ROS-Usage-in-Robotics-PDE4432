#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16

pub = rospy.Publisher('lcd_msg', Int16, queue_size=10)

def talker():
    taskNum = rospy.Publisher('taskNumber', Int16, queue_size=10)
    rospy.Subscriber("distance", Int16, callback)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.5)
    count = 0
    
    #print("Task number '1' published to topic taskNumber")
    while not rospy.is_shutdown():
        taskNum.publish(3)
        rate.sleep()

def callback(data):
   pub.publish(data.data)
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
