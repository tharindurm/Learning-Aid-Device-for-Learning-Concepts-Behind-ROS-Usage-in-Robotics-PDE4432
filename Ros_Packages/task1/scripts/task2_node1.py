#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16

def talker():
    pub = rospy.Publisher('lcd_msg', Int16, queue_size=10)
    taskNum = rospy.Publisher('taskNumber', Int16, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.5)
    count = 0
    
    #print("Task number '1' published to topic taskNumber")
    while not rospy.is_shutdown():
        taskNum.publish(1)
        pub.publish(count)
        
        print("Published number ",str(count)," to ROS topic '/lcd_msg'")
        if count == 10:
            count = 0
            print("---------------------------------------------- Restarting Loop ----------") 
        else:
            count=count+1
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
