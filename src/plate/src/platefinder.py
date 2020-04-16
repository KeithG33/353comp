#!/usr/bin/env python
import cv2
import numpy as np
import pytesseract
import re
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32MultiArray, Int32
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError

image_topic =  '/rrbot/camera1/image_raw'
debounce_time = rospy.Duration(1)
cancel_time = rospy.Duration(2)

class boxgetter():

    def __init__(self, kernel_size = (10,1), cut_max = (200,200,50), cut_min=(0, 0,0), **kwargs):

        self.send_number = 3
        self.sent = 0
        self.kernel = np.ones(kernel_size, np.uint8)
        self.cut_max = cut_max
        self.cut_min = cut_min
        self.available = True
        self.bridge = CvBridge()
        self.sync_pub = rospy.Publisher('sync_num', Int32, queue_size=6)
        self.imgs_pub = rospy.Publisher('extracted_plates', Image, queue_size=12)
        self.time_out = None
        #rospy.init_node('extracted_plates')
        rospy.init_node('extract_plates', anonymous=True)


    def get_rois(self, image):
        
        ret_list = []
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        thresh = cv2.inRange(hsv, self.cut_min,self.cut_max)
        img_dilation = cv2.dilate(thresh, self.kernel, iterations=1)


        # find contours
        # cv2.findCountours() function changed from OpenCV3 to OpenCV4: now it have only two parameters instead of 3
        cv2MajorVersion = cv2.__version__.split(".")[0]
        # check for contours on thresh
        if int(cv2MajorVersion) >= 4:
            ctrs, hier = cv2.findContours(img_dilation.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        else:
            im2, ctrs, hier = cv2.findContours(img_dilation.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # sort contours
        
        sorted_ctrs = sorted(ctrs, key=lambda ctr: cv2.boundingRect(ctr)[0])

        for i, ctr in enumerate(sorted_ctrs):
            # Get bounding box
            x, y, w, h = cv2.boundingRect(ctr)

            # Getting ROI
            roi = thresh[y:y + h, x:x + w]

            # show ROI
            # cv2.imshow('segment no:'+str(i),roi)
            #cv2.waitKey(0)
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
            if w > 50 and h > 30 and w>h and 1.0*h/w<2.0/3*1.2 and 1.0*h/w>2.0/3*0.8: #more checks here
                #ret_list.append(roi)
                
                letter = thresh[y+int(h*0.15):y+int(h*0.85),x+int(w*0.02):x+int(w*0.98)]
                # cv2.imshow('letter'+str(i), letter)
                #cv2.waitKey(0)
                ret_list.append(letter)
    
    #     cv2.imshow('i', image)
    #    # cv2.waitKey(0)
        return ret_list#[x for x in ret_list if type(x) == np.ndarray and 0 not in x.shape]

    def get_image(self, data):
        return np.fromstring(data.data, dtype='uint8').reshape((data.height, data.width, 3))



    def publish_extracted_plates(self, data):
        #rospy.loginfo(pub_str)

        self.sync_pub.publish(len(data))

        for d in data:
            inverted = 255-d
            img = cv2.cvtColor(inverted, cv2.COLOR_GRAY2RGB)
            msg = self.bridge.cv2_to_imgmsg(img,'rgb8')
            self.imgs_pub.publish(msg)
        
    def set_available(self, *args):
        self.sent = 0 

    

    def callback(self, data):
        
        image = self.get_image(data)
        plates = self.get_rois(image)
        print(self.sent)
        if len(plates) == 2 and self.sent < self.send_number:
            if self.time_out == None:
                self.time_out = rospy.Timer(cancel_time, self.set_available, oneshot=True)

            print("sending")
            self.sent += 1
            self.publish_extracted_plates(plates)
            if self.sent == self.send_number:
                self.time_out.shutdown()
                self.time_out = None
                rospy.Timer(debounce_time, self.set_available, oneshot=True)
        
            


    def start(self):
        rospy.Subscriber(image_topic, Image, self.callback)
        rospy.spin()

if __name__ == "__main__":
    b = boxgetter()
    b.start()

    # image = cv2.imread("image2.png")
    
    
    # plates = b.get_rois(image)

    # letters = b.get_numbers(plates)
    # print(letters)
