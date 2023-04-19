#!/usr/bin/env python
#~/catkin_ws/src/race/src/control.py

import rospy
from race.msg import drive_param, pid_input
from std_msgs.msg import String
import time


kp = 250.0
kd = 0.0
servo_offset = 18.5
prev_error = 0.0
vel_input = 0.0
ang_input = 0.0
start_time = 0.0
choice = 0  # choose spin mode
# end_time = time.monotonic()


def callback(data, pub):
    global ang_input, vel_input, end_time
    # TEST: show time elapsed between callbacks
    # new_start = time.monotonic()
    # elapsed = new_start - end_time
    # print(f"Elapsed time between callback: {elapsed:.5f}s")

    angle = ang_input
    velocity = vel_input

    msg = drive_param()

    if angle > 100:
        angle = 100
    if angle < -100:
        angle = -100

    msg.velocity = velocity
    msg.angle = angle

    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)

    print("Velocity: " + msg.velocity + " Angle: " + msg.angle)
    pub.publish(msg)

    # TEST:
    # end_time = time.monotonic()


def command_subscriber():
    rospy.init_node('command_subscriber', anonymous=True)
    rospy.Subscriber("command_topic", String, callback)
    rospy.spin()

def main():
    # ros.Subscriber("vision/imageStuff", Image, depth_callback, (pub))
    global ang_input, vel_input	
    pub_queue_size = 1
    sub_queue_size = 1
	# choice = 0
    given_time = 100
    hz = 10
    # command
    # command_s1
    # command_s2
        # # Get user input
    # # print("Listening to error for PID")
    # # vel_input = float(input("Enter Velocity: "))
    # """
    # # Choose different types of spin
    # print("Choose spinning method(0--Robust Spin, 1--Dynamic Spin, 2--Static Spin): ")
    # choice = int(input())
    # if choice == 1:
    #     print("Enter Execution Time in milliseconds: ")
    #     given_time = int(input())
    # if choice == 2:
    #     print("Enter Frequency in HZ: ")
    #     hz = int(input())
    # """

    # Initiate Node    
    rospy.init_node("pid_controller")

    # Create Node Handler for publisher
    pub_queue_size = 1
    pub = rospy.Publisher("drive_parameters", drive_param, queue_size=pub_queue_size)

    # Create Node Handler for subscriber
    sub_queue_size = 1
    rospy.Subscriber("error", pid_input, callback, pub)

    run = True
    while run:
        msg = drive_param()
        command = raw_input("Enter command: ")
        command_type = command[0]

        if len(command) > 1:
            value = float(command[1:])
        else:
            value = 0

        if command_type == "s":
            vel_input = value
            print("Velocity set to " + str(vel_input))
        elif command_type == "t":
            ang_input = value
            print("Angle set to " + str(ang_input))
        elif command_type == "c":
            vel_input = 12
            ang_input = 100
            print("Velocity set to " + str(vel_input) + " and Angle set to " + str(ang_input))
        elif command_type == "h":
            vel_input = 0
            ang_input = 0
            print("Car stopped")
        elif command_type == "q":
            vel_input = 0
            ang_input = 0
            print("Car stopped. Exiting...")
            run = False
        else:
            vel_input = 0
            ang_input = 0
            print("Default case. Stop the car and exit.")
            run = False

        msg.velocity = vel_input
        msg.angle = ang_input
        # TODO Check angle
        pub.publish(msg)
        # CHECKS CALLBACK once
        # rospy.spin()
        # command_subscriber()

if __name__ == '__main__':
	print("RUNNING")
	main()
