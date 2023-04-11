#!/usr/bin/env python

# ~/catkin_ws/src/race/src/hw1sub.py

import rospy
from std_msgs.msg import String

def pp(s):
    rospy.loginfo(rospy.get_caller_id() + "%s", str(str))

def chatterCallback(data):
    print(data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("CS378TEST", String, chatterCallback)
    rospy.spin()

if __name__ == '__main__':
    print("RUNNING")
    pp("RUNNNIG1")
    listener()