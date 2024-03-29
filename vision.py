#!/usr/bin/env python
# ~/catkin_ws/src/color_tracking/src/vision.py
import rospy as ros
import cv2 as cv
import numpy as np
import json
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

ros.init_node("image_listener", anonymous=True)
bridge = CvBridge()
curr_circles = []
class circle:
    def __init__(self, center, radius, score):
        self.center = center
        self.radius = radius
        self.score = score
        self.depth = None

    def x(self):
        return self.center[0]

    def y(self):
        return self.center[1]
    
    def set_depth(self, val):
        if val and not np.isnan(val):
            self.depth = val

    def get_depth(self):
        if self.depth:
            return self.depth
        else:
            return -1.0

    def rank(self):
        points = 0
        if self.radius > 50 and self.radius < 180:
            points += 1
        if self.score > 0.4:
            points += int((self.score*10-4)*2)
        if self.get_depth() > 500 and self.get_depth() < 4000:
            points += 1
        if self.x() > 1280/4 and self.x() < 1280/4*3 and self.y() > 720/4 and self.y() < 720/4*3:
            points += 1

        return points
    
    def __str__(self):
        return "Center: "+str(self.center)+ " Radius: "+ str(self.radius)+ " Score: " + str(self.score) + " Depth: " + str(self.get_depth()) + " Rank: " + str(self.rank())
    
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
    #print("img_callback")
    img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    color_img = img
    color_recv = True
    #DEBUG DISPLAY
    if True:
        disp = img.copy()
        for c in curr_circles:
            d = int(c.radius*.4)
            tl = (c.x()-d, c.y()-d)
            br = (c.x()+d, c.y()+d)
            cv.rectangle(disp, tl, br, (255,0,0),1)
            cv.circle(disp, c.center, c.radius, (0,0,255), 3)
            cv.putText(disp, str(c.get_depth()) + " R" + str(c.rank()), c.center, cv.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 1)
    # try:
    #    #cv.imshow("color", disp)
    #    #cv.waitKey(1)
    # except Exception:
    #     pass
    detect(pub)


def depth_callback(msg, pub):
    global depth_recv
    global depth_img
    #print("depth callback")
    img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    #DEBUG DISPLAY
    if False:
        cv.imshow("depth Colored", img)
        mask = cv.inRange(img, 0,0)
        for c in curr_circles:
            cv.circle(mask, c.center, c.radius, 150, 3)
        cv.imshow("o", mask)
        cv.waitKey(1)
    depth_img = img
    depth_recv = True


def estimate_depth(depth, circle):
    d = int(circle.radius*.4)
    tl = (circle.x()-d, circle.y()-d)
    br = (circle.x()+d, circle.y()+d)
    chunk = depth[tl[1]:br[1], tl[0]:br[0]]
    return np.sum(chunk)/np.count_nonzero(chunk)

def choose_circle(circles):
    return sorted(circles, key=lambda c: c.rank())[0]

kick_state = 0
frames_in_state = 0
def send_instruction(pub, circle):
    if not circle:
        return
    global kick_state, frames_in_state
    vel = 0
    CLOSE = 950
    MIN_SPEED = 18
    SWEET_SPOT = 1280//2 #what pixel of the center of screen is appropriate
    if circle.get_depth() > CLOSE:
        if kick_state == 1:
            kick_state = 0
            frames_in_state = 0
        frames_in_state +=1

        vel = (circle.get_depth() - CLOSE) / 75 + MIN_SPEED
    elif circle.get_depth() <= CLOSE and abs(circle.x() - SWEET_SPOT) < 213:
        if kick_state == 0:
            kick_state = 1
            frames_in_state = 0
        frames_in_state +=1
        vel = 90
    if frames_in_state > 10 and kick_state == 1:
        #we've been kicking for a while
        # stop for a while
        frames_in_state +=1
        vel = 0
        if frames_in_state > 200:
            frames_in_state = 0
    

    angle = (circle.x() - SWEET_SPOT) / 5
    pub.publish(json.dumps({'speed':vel, 'angle':angle}))

frames_since = 0
def detect(pub):
    global curr_circles, frames_since
    if color_recv and depth_recv:
        hsv_img = cv.cvtColor(color_img, cv.COLOR_BGR2HSV)
        contour_circles = circle_detect_contours(hsv_img, 0.4)
        for c in contour_circles:
            c.set_depth(estimate_depth(depth_img, c))
        if contour_circles:
            frames_since = 0
            curr_circles = contour_circles
        else:
            print "NONE FOUND"
        circle = choose_circle(curr_circles)
    	    #logic to publish is here
        if circle:
            print "Rank " + str(circle.rank()) + " frames since " + str(frames_since)
        frames_since += 1

        CONSEC = 5
        if circle.rank() > 2 and frames_since < CONSEC:
            send_instruction(pub, circle)
        else:
            print "STOPPING"
            pub.publish(json.dumps({'speed':0, 'angle':-20}))
        
        #for c in curr_circles:
            #print c
        
    

        


def main():
    pub = ros.Publisher("instruct/commands", String, queue_size=1000)
    pub.publish("s 0")
    
    ros.Subscriber("depth/depth_raw_registered", Image, depth_callback, (pub))
    ros.Subscriber("rgb/image_rect_color", Image, img_callback, (pub))
    
    ros.spin()

if __name__ == '__main__':
    print("RUNNING")
    main()
