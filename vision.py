#!/usr/bin/env python

import rospy as ros
import cv2 as cv


def main():
    ros.init_node("image_listener", anonymous=True)
    sub = ros.Subscriber("rgb/image_rect_color")


if __name__ == '__main__':
    print("RUNNING")
    main()
