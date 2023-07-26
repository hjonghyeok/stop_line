#! /usr/bin/env python3

import cv2
from std_msgs.msg import String
import rospy
import numpy as np
import math
from std_msgs.msg import Int32


cap = cv2.VideoCapture("/home/jh/catkin_ws/src/stopline/scripts/driving.mp4")

rospy.init_node("stopline_node", anonymous=True)
pub = rospy.Publisher("stopline", Int32, queue_size = 10)

X,Y,W,H = 300,300,600,1000 # roi setting start position x,y, width, height

def roi_image(image):

    (x,y),(w,h) = (X,Y),(W,H) 
    roi_img = image[y:y+h, x:x+w]

    return roi_img


def white_filter(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    white_lower = np.array([0, 0, 200]) 
    white_upper = np.array([100, 100, 255])

    white_mask = cv2.inRange(hsv, white_lower, white_upper)

    white_masked = cv2.bitwise_and(image, image, mask=white_mask)

    return white_masked


def line_write(stop_lines):   # line detection
    if stop_lines is not None:
        for i in range(0, len(stop_lines)):
            rho = stop_lines[0][0][0]
            theta = stop_lines[0][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
            pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a))) 
            cv2.line(white_cdst, pt1, pt2, (30, 225, 225), 2, cv2.LINE_AA)
            pub.publish(1)
    else:
        pub.publish(0)


            



while not rospy.is_shutdown():

    ret, src1 = cap.read()
    src = src1.copy()
    src=roi_image(src) 
    
    cv2.rectangle(src1, (X, Y), (X+W, Y+H), (255,0,0), 5)
    white_src = white_filter(src)

    white_dst = cv2.Canny(white_src, 300, 600, None, 3)

    white_cdst = cv2.cvtColor(white_dst, cv2.COLOR_GRAY2BGR) 
    white_cdstP = np.copy(white_cdst)

    stop_lines = cv2.HoughLines(white_dst, 1, 5*np.pi / 180, 250, None, 0, 0, 82, 98)

    line_write(stop_lines)



    cv2.imshow("Source", src1)
    cv2.imshow("Stop Lines (in red) - Standard Hough Line Transform", white_cdst)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release(10)
cv2.destoryAllWindows()
