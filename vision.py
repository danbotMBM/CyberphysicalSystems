#!/usr/bin/env python
# ~/catkin_ws/src/color_tracking/src/vision.py
import rospy as ros
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

ros.init_node("image_listener", anonymous=True)
bridge = CvBridge()

class circle:
    def __init__(self, center, radius, score):
        self.center = center
        self.radius = radius
        self.score = score

    def x(self):
        return self.center[0]

    def y(self):
        return self.center[1]
    
    def __str__(self):
        return "Center: "+str(self.center)+ " Radius: "+ str(self.radius)+ " Score: " + str(self.score)
    
def red_mask(hsv_img):
    #must be HSV
    lower_red1 = np.array([0, 100, 20])
    lower_red2 = np.array([9, 255, 255])
    upper_red1 = np.array([171, 100, 20])
    upper_red2 = np.array([180, 255, 255])
    mask1 = cv.inRange(hsv_img, lower_red1, lower_red2)
    mask2 = cv.inRange(hsv_img, upper_red1, upper_red2)
    return mask1 | mask2

def draw_circles(img, circles, color):
    for c in circles:
        cv.circle(img, c.center, c.radius, color, 2)

def morph(mask):
    kernel = np.ones((5, 5), np.uint8)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
    # cv.imshow('mask', mask)
    
    # cv.waitKey(1)

def circle_detect_hough(bgr_img):
    gray = cv.cvtColor(bgr_img, cv.COLOR_BGR2GRAY)

    circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=0, maxRadius=0)

    result = []
    if circles is not None:
        r_circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in r_circles:
            result.append(circle((x,y), r, 0))
    return result


def circle_detect_contours(img, threshold):
    RADIAL_MIN = 15
    mask = red_mask(img)
    morph(mask)

    contours = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    #print("contours", contours)
    circles = []
    for cnt in contours:
        area = cv.contourArea(cnt)
        perimeter = cv.arcLength(cnt, True)
        if perimeter == 0:
            continue
        circularity = 4 * np.pi * area / (perimeter ** 2)
        if circularity > threshold:
            (x, y), radius = cv.minEnclosingCircle(cnt)
            center = (int(x), int(y))
            radius = int(radius)
            if radius > RADIAL_MIN:
                circles.append(circle(center, radius, circularity))
    return circles


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
    #cv.imshow("Colored", img)
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
    if color_recv and depth_recv:
        #print(color_img.shape, type(color_img[0][0][0]), color_img)
        #print(depth_img.shape, type(depth_img[0][0]), depth_img)
    	#cv.imshow("depth", depth_img)
    	#cv.imshow("color", color_img)
        hsv_img = cv.cvtColor(color_img, cv.COLOR_BGR2HSV)
        contour_circles = circle_detect_contours(hsv_img, 0.4)
        for c in contour_circles:
            if c.x() < 720 and c.x() > 0 and c.y() < 1280 and c.y() > 0:
                print(str(c), "depth=", depth_img[c.x()][c.y()])
        


def main():
    pub = ros.Publisher("vision/imageStuff", String, queue_size=1000)
    
    ros.Subscriber("depth/depth_raw_registered", Image, depth_callback, (pub))
    ros.Subscriber("rgb/image_rect_color", Image, img_callback, (pub))
    
    ros.spin()

if __name__ == '__main__':
    print("RUNNING")
    main()
