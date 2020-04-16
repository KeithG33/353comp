#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import imutils
import matplotlib.pyplot as plt

l_blu = np.array([120,70,70]) 
u_blu = np.array([150,255,255])

l_yellow = np.array([30,50,50])
u_yellow = np.array([50,255,255])

l_green = np.array([50,50, 50])
u_green = np.array([90, 255, 255])

class car_image_publisher():
    def __init__(self):
        self.img_pub = rospy.Publisher("car_detector",Image,queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,self.callback)

    def is_car_detected(self, img):
        global l_blu 
        global u_blu 
        global l_yellow
        global u_yellow
        global l_green
        global u_green
        height = img.shape[0]
        width = img.shape[1]

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # ROI_right = img[height-400:height,width-300:width-50]
        # ROI_left = img[height-400:height, 50:300]

        mask_green = cv2.inRange(hsv, l_green, u_green)
        res_green = cv2.bitwise_and(img,img, mask= mask_green)
        # cv2.imshow("see",res_green)
        # cv2.waitKey(0)
        mask_blue = cv2.inRange(hsv, l_blu, u_blu)
        res_blue = cv2.bitwise_and(img,img, mask= mask_blue)
        # cv2.imshow("see",res_blue)
        # cv2.waitKey(0)
        mask_yellow = cv2.inRange(hsv, l_yellow, u_yellow)
        res_yellow = cv2.bitwise_and(img,img, mask= mask_yellow)
        # cv2.imshow("see",res_yellow)
        # cv2.waitKey(0)

        greyd_green = cv2.cvtColor(res_green,cv2.COLOR_BGR2GRAY)
        greyd_blue = cv2.cvtColor(res_blue,cv2.COLOR_BGR2GRAY)
        greyd_yellow = cv2.cvtColor(res_yellow,cv2.COLOR_BGR2GRAY)

        cont_green = cv2.findContours(greyd_green, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        cont_green = imutils.grab_contours(cont_green)
        cont_blue = cv2.findContours(greyd_blue, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        cont_blue = imutils.grab_contours(cont_blue)    
        cont_yellow = cv2.findContours(greyd_yellow, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        cont_yellow = imutils.grab_contours(cont_yellow)   

        if len(cont_green) != 0:
            biggest = max(cont_green,key=cv2.contourArea)
            area = cv2.contourArea(biggest)/100
            if area > 250:
                print "green car nearby"
                return True
        if len(cont_blue) != 0:
            biggest = max(cont_blue,key=cv2.contourArea)
            area = cv2.contourArea(biggest)/100
            if area > 250:
                print "blue car nearby"
                return True
        if len(cont_yellow):
            biggest = max(cont_yellow,key=cv2.contourArea)
            area = cv2.contourArea(biggest)/100
            if area > 250:
                print "yellow car nearby"
                return True
        return False


    def callback(self,data):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        if cv_img is not None:
            is_car_close = self.is_car_detected(cv_img)

            if is_car_close == True:
                cv_img_msg = self.bridge.cv2_to_imgmsg(cv_img,'bgr8')
                self.img_pub.publish(cv_img_msg)

def main(args):
    detector = car_image_publisher()
    rospy.init_node('detect', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
