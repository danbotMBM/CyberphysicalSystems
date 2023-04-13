#!/usr/bin/env python
# ~/catkin_ws/src/color_tracking/src/vision.py
import rospy as ros
import cv2 as cv
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

ros.init_node("image_listener", anonymous=True)
bridge = CvBridge()

color_recv = False
depth_recv = False
color_img = None
depth_img = None

def img_callback(msg, pub):
    global color_img
    global color_recv
    print("img_callback")
    img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    color_img = img
    color_recv = True
    #print(img.shape, img)
    display()


def depth_callback(msg, pub):
    global depth_recv
    global depth_img
    print("depth callback")
    img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    depth_img = img
    depth_recv = True
    #print(img.shape, type(img[0][0]))
    #print(img)
    display()
    #img = cv.normalize(img, None, 0, 255, cv.NORM_MINMAX, cv.CV_8UC1)

    # map = cv.applyColorMap(img, cv.COLORMAP_RAINBOW)
    #cv.imshow("depth Colored", img)
    #cv.waitKey(1)


def display():
    depth_recv = True
    if color_recv and depth_recv:
        print(color_img.shape, type(color_img[0][0][0]), color_img)
        print(depth_img.shape, type(depth_img[0][0]), depth_img)
    	#cv.imshow("depth", depth_img)
    	#cv.imshow("color", color_img)
  
        
    cv.waitKey(1)


def main():
    pub = ros.Publisher("vision/imageStuff", String, queue_size=1000)
    
    ros.Subscriber("depth/depth_raw_registered", Image, depth_callback, (pub))
    ros.Subscriber("rgb/image_rect_color", Image, img_callback, (pub))
    
    ros.spin()

if __name__ == '__main__':
    print("RUNNING")
    main()
