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

count=0
#image_amount_modifier=100


#Here we'll have an initiallize method and the callback method, which checks the current movement
#from /cmd_vel publisher. Then it saves the camera image to the folder corresponding to it's action:
#left, right, forward, backward. In other words, generating data for CNN for road driving.
class image_converter:

    def __init__(self):
        self.current_image = None
        self.bridge = CvBridge()
        self.move = Twist()
        self.linear_speed = None
        self.angular_speed = None
        
        
    def image_callback(self,data):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            print("we here...")
            #cv2.imshow("test", current_image)
            #cv2.waitKey(2)

            if not self.linear_speed  == 0 or not self.angular_speed == 0 and not self.current_image == None:
                if self.linear_speed > 0:
                    self.current_action = "forward"
                    #cv2.imwrite('/home/fizzer/353comp/src/imitation_pics/forward/forward_%d' %count,cv_image)
                    print("Loaded image into folder: /home/fizzer/.../forward")
                elif self.linear_speed < 0:
                    action = "backward"
                    #cv2.imwrite('/home/fizzer/353comp/src/imitation_pics/backward/backward_%d' %count,cv_image)
                    print("Loaded image into folder: /home/fizzer/.../backward")
                elif self.angular_speed > 0:
                    action = "left"
                    #cv2.imwrite('/home/fizzer/353comp/src/imitation_pics/left/left_%d' %count,cv_image)
                    print("Loaded image into folder: /home/fizzer/.../right")
                elif self.angular_speed < 0:
                    action = "right"
                    #cv2.imwrite('/home/fizzer/353comp/src/imitation_pics/right/right_%d' %count,cv_image)
                    print("Loaded image into folder: /home/fizzer/.../left")
        
                print(action)
            else:
                print("Not moving")
        except CvBridgeError as e:
            print(e)


    def callback(self,data):

        print("here now too")
        global count

        self.linear_speed = data.linear.x
        self.angular_speed = data.angular.z

        print(self.linear_speed)
        print(self.angular_speed)

        count = count + 1
        print(count)


def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    move_sub = rospy.Subscriber("/cmd_vel", Twist, ic.callback)
    image_sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,ic.image_callback)
    rospy.sleep(1)

    try:
       rospy.spin()
    except KeyboardInterrupt:
       print("Shutting down")
       cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

            


