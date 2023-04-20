#!/usr/bin/env python
#~/catkin_ws/src/race/src/control.py

import rospy
from race.msg import drive_param, pid_input
from std_msgs.msg import String
import time
import json


kp = 250.0
kd = 0.0
servo_offset = 18.5
prev_error = 0.0
vel_input = 0.0
ang_input = 0.0
start_time = 0.0
choice = 0  # choose spin mode
sub_node = None
# end_time = time.monotonic()


def intruction_callback(data, pub):
    print "HEARD " + str(data)
    global ang_input, vel_input, end_time
    msg = parse_command(data.data)
    print("Velocity: " + str(msg.velocity) + " Angle: " + str(msg.angle))
    pub.publish(msg)


def parse_command(command):
    global ang_input, vel_input
    msg = drive_param()
    params = json.loads(command)
    vel_input = params["speed"]
    ang_input = params["angle"]
    msg.velocity = vel_input
    msg.angle = ang_input
    return msg    

def main():
    # Initiate Node    
    rospy.init_node("pid_controller")

    # Create Node Handler for publisher
    pub_queue_size = 1
    pub = rospy.Publisher("drive_parameters", drive_param, queue_size=1)

    # Create Node Handler for subscriber
    sub_queue_size = 1
    subnode = rospy.Subscriber("instruct/commands", String, intruction_callback, pub)
    rospy.spin()

if __name__ == '__main__':
	print("RUNNING")
	main()
