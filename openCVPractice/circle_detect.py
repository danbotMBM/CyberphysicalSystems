
# importing cv
import cv2 as cv
import numpy as np

cap = cv.VideoCapture(0)

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
        return f"Center: {str(self.center)}, Radius: {self.radius}, Score: {self.score}"
    
def red_mask(hsv_img):
    #must be HSV
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])
    mask = cv.inRange(hsv_img, lower_red, upper_red)
    return mask

def draw_circles(img, circles, color):
    for c in circles:
        cv.circle(img, c.center, c.radius, color, 2)

def morph(mask):
    kernel = np.ones((5, 5), np.uint8)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
    cv.imshow('mask', mask)

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
    RADIAL_MIN = 6
    mask = red_mask(img)
    morph(mask)

    contours, hierarchy = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
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

while True:
    ret, img = cap.read()
    hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    contour_circles = circle_detect_contours(hsv_img, 0.8)
    for c in contour_circles:
        print(c, end="")
    print()
    #hough_circles = circle_detect_hough(img)
    drawn = img.copy()
    blue = (255, 0, 0)
    green = (0, 255, 0)
    draw_circles(drawn, contour_circles, blue)
    #draw_circles(drawn, hough_circles, green)
    cv.imshow('Circles', drawn)
    cv.waitKey(1)
 
cv.destroyAllWindows()