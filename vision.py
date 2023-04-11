#!/usr/bin/env python
# ~/catkin_ws/src/color_tracking/src/vision.py
import rospy as ros
import cv2 as cv
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

ros.init_node("image_listener", anonymous=True)
bridge = CvBridge()

def img_callback(msg, pub):
    print("callback")
    img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    cv.imshow("name", img)
    cv.waitKey(1)

def depth_callback(msg, pub):
    print("callback")
    img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    print(img.shape, type(img[0][0]))
    img = cv.normalize(img, None, 0, 255, cv.NORM_MINMAX, cv.CV_8UC1)
    cv.imshow("name", img)
    map = cv.applyColorMap(img, cv.COLORMAP_RAINBOW)
    cv.imshow("depth Colored", map)
    cv.waitKey(1)


def main():
    pub = ros.Publisher("vision/imageStuff", String, queue_size=1000)
    sub = ros.Subscriber("depth/depth_raw_registered", Image, depth_callback, (pub))
    ros.spin()

if __name__ == '__main__':
    print("RUNNING")
    main()
