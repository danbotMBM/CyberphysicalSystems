
# importing cv2
import cv2
import numpy as np

cap = cv2.VideoCapture(0)

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
    mask = cv2.inRange(hsv_img, lower_red, upper_red)
    return mask

def draw_circles(img, circles, color):
    for c in circles:
        cv2.circle(img, c.center, c.radius, color, 2)

def morph(mask):
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    cv2.imshow('mask', mask)

def circle_detect_hough(bgr_img):
    gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)

    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=0, maxRadius=0)

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

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    circles = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0:
            continue
        circularity = 4 * np.pi * area / (perimeter ** 2)
        if circularity > threshold:
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            center = (int(x), int(y))
            radius = int(radius)
            if radius > RADIAL_MIN:
                circles.append(circle(center, radius, circularity))
    return circles

while True:
    ret, img = cap.read()
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
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
    cv2.imshow('Circles', drawn)
    cv2.waitKey(1)
 
cv2.destroyAllWindows()