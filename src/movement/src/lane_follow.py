#!/usr/bin/env python
from geometry_msgs.msg import Twist
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


last_error = 0
Kp = 0.08
Kd = 0.01
cruising_speed = 0.15
current_portion = "first_straight"
straight2_time = 0
straight3_time = 0
straight4_time = 0
middle_time = 0


class get_yo_ass_in_lane():

    def __init__(self):
        self.move_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,self.callback)
        self.move = Twist()
    
    # Buncha helper functions first
    def lane_drive(self, error):
        global last_error
        turnspeed = Kp*error + Kd*(error - last_error)
        #centre of lane to the left
        if error > 0:
            if error >= 40:
                self.set_speeds(cruising_speed/1.5, turnspeed/40)
            else:
                self.set_speeds(cruising_speed, turnspeed/15)
            last_error = error
        elif error < 0:
            if error <= -40:
                self.set_speeds(cruising_speed/1.5,turnspeed/40)
            else:
                self.set_speeds(cruising_speed, turnspeed/15)
            last_error = error

    def right_turn(self):
        self.move.linear.x = 0.15
        self.move_pub.publish(self.move)
        rospy.sleep(9.2)
        self.move.linear.x = 0
        self.move_pub.publish(self.move)
        self.set_speeds(0,-0.48)
        rospy.sleep(7.5)

    def right_turn_2(self):
        self.move.linear.x = 0.16
        self.move_pub.publish(self.move)
        rospy.sleep(7.1)
        self.set_speeds(0,-0.6)
        rospy.sleep(5.8)
    
    def right_turn_3(self):
        self.move.linear.x = 0.15
        self.move_pub.publish(self.move)
        rospy.sleep(8.4)
        self.set_speeds(0,-0.6)
        rospy.sleep(6.1)
    
    def right_turn_4(self):
        self.set_speeds(0.15,0)
        rospy.sleep(7.9)
        self.set_speeds(0,0)
        self.set_speeds(0,-0.6)
        rospy.sleep(7.8)

    def find_ROI(self, img):
        height = img.shape[0]
        width = img.shape[1]
        mask = np.zeros_like(img,dtype=np.uint8)
        vertices = np.array( [[(0,height), (0,height-95),  (width/2,height-115), (width,height-95), (width, height)]])
        cv2.fillPoly(mask, vertices, 255)
        ROI = cv2.bitwise_and(img, mask)
        return ROI

    def do_canny(self,img):
        ROI_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ROI_blur = cv2.GaussianBlur(ROI_gray,(5,5),0)
        canny = cv2.Canny(ROI_blur,50,150)
        return canny

    def calculate_coordinates(self,img, parameters):
        slope, intercept = parameters
        y1 = img.shape[0]
        y2 = int(y1 - 50)
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)
        return np.array([x1, y1, x2, y2])

    def calculate_lines(self, img,lines):
        left = []
        right = []
        width = img.shape[1]

        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            # Fits with a linear polynomial  and returns slope and y-intercept

            if x1 < 180 or x1 > (width - 180):
                parameters = np.polyfit((x1, x2), (y1, y2), 1)
                slope = parameters[0]
                y_intercept = parameters[1]
                # If slope is negative, the line is to the left, otherwise line is to the right
                if slope < -0.6:
                    left.append((slope, y_intercept))
                elif slope > 0.6 :
                    right.append((slope, y_intercept))

        lines = []
        if len(left) != 0 and len(right) != 0:
            left_avg = np.average(left, axis = 0)
            left_line = self.calculate_coordinates(img, left_avg)
            right_avg = np.average(right, axis = 0)
            right_line = self.calculate_coordinates(img, right_avg)
            return np.array([left_line,right_line])
        elif len(left) != 0:
            left_avg = np.average(left, axis = 0)
            left_line = self.calculate_coordinates(img, left_avg)
            lines.extend(left_line)
        elif len(right) != 0:
            right_avg = np.average(right, axis = 0)
            right_line = self.calculate_coordinates(img, right_avg)
            lines.extend(right_line)
        return np.array([lines])

    def set_speeds(self, linear, angular):
        self.move.linear.x = linear
        self.move.angular.z = angular
        self.move_pub.publish(self.move)
        

# Stays in lane by detecting vertical lines on either side of road. Uses canny edge detection, 
# then Hough Line transform to get slope and coordinates
#
# Uses a state variable for what part of course it's in (sorry Miti...)
    def callback(self,data):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        global cruising_speed
        global last_error
        global current_portion
        global straight2_time
        global straight3_time
        global straight4_time
        global middle_time

        right = False
        left = False
        both = False
        height = cv_img.shape[0]
        width = cv_img.shape[1]
        setpoint = width/2

        #image stuffs
        canny = self.do_canny(cv_img)
        ROI = self.find_ROI(canny)
        hough = cv2.HoughLinesP(ROI, 3, np.pi / 180, 100, np.array([]), minLineLength = 50, maxLineGap = 5)
   

        if hough is not None:
            lines = self.calculate_lines(cv_img, hough)
            lines_visualize = np.zeros_like(cv_img)
            x_sum = 0
            # Checks if any lines are detected, average x's to draw dot 
            if len(lines) != 0 or lines is not None or lines != []:
                left_x_avg=0
                right_x_avg = 0
                if len(lines) == 2:
                    both = True
                for x1, y1, x2, y2 in lines:
                    if x1 < width/2:
                        left_x_avg = (x1+x2)/2
                    elif x1 > width/2:
                        right_x_avg = (x1+x2)/2
                    if both == False:
                        if x1 < width/2:
                            left = True 
                        elif x1 > width/2:
                            right = True
                    x_sum += x1
                    cv2.line(lines_visualize, (x1, y1), (x2, y2), (0, 255, 0), 5)
                if left_x_avg != 0 :
                    cv2.circle(lines_visualize,(left_x_avg+215,height - 50), 8,(0,255,255),-1)
                x_avg = x_sum/2
                cv2.circle(lines_visualize, (x_avg,height - 50),8,(255,15,255),-1)
                output = cv2.addWeighted(cv_img, 0.9, lines_visualize, 1, 1)
                cv2.imshow("finaloutput", output)
                cv2.waitKey(2)

            error = (setpoint - x_avg)/10
            if left_x_avg != 0:
                left_error = (setpoint - (left_x_avg+215)) / 2 
                print("LEFT error is....{}".format(left_error))
            if right_x_avg != 0:
                right_error = (setpoint - (right_x_avg-205)) / 2 
                print("RIGHT error is....{}".format(right_error))

            #centre of lane is to the left
            if current_portion == "first_straight":
                self.lane_drive(left_error)
                
                if left == True:
                    now = rospy.get_time()
                    if now > 35:
                        self.right_turn()
                        current_portion = "second_straight"
                        straight2_time = rospy.get_time()
            elif current_portion == "second_straight":  
                self.lane_drive(left_error)
                if left == True:
                    now = rospy.get_time()
                    if now - straight2_time > 25:
                        self.set_speeds(cruising_speed,0)
                        self.right_turn_2()
                        current_portion = "third_straight"
                        straight3_time = rospy.get_time()
            elif current_portion == "third_straight":
                self.lane_drive(left_error)
                if left == True:
                    now = rospy.get_time()
                    if now - straight3_time > 20:
                        self.set_speeds(cruising_speed,0)
                        self.right_turn_3()
                        current_portion = "fourth_straight"
                        straight4_time = rospy.get_time()  
            elif current_portion == "fourth_straight":
                self.lane_drive(left_error)
                if left == True:
                    self.lane_drive(left_error)
                    now = rospy.get_time()
                    if now - straight4_time > 10:
                        self.set_speeds(cruising_speed,0)
                        self.right_turn()
                        current_portion = "middle"
                        middle_time = rospy.get_time()
            elif current_portion == "middle":
                now = rospy.get_time()
                if left == True:
                    
                    self.lane_drive(left_error)
                elif both == True:
                    self.lane_drive(error)
                elif right == True:
                    self.lane_drive(right_error)
                
                if now - middle_time > 26: 
                    current_portion = "doneski"
            elif current_portion == "doneski":
                print "made it"
                self.set_speeds(0,-0.3)


            
def main(args):
  driver = get_yo_ass_in_lane()
  rospy.init_node('drive', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
