#!/usr/bin/env python
import cv2
import numpy as np
import pytesseract
import re
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

image_topic =  '/rrbot/camera1/image_raw'
debounce_time = rospy.Duration(0.1)

class boxgetter():

    def __init__(self, kernel_size = (10,1), cut_max = (200,200,50), cut_min=(0, 0,0), **kwargs):
        self.kernel = np.ones(kernel_size, np.uint8)
        self.cut_max = cut_max
        self.cut_min = cut_min
        self.available = True
        self.pub = rospy.Publisher('plate', String, queue_size=10)
        rospy.init_node('plate', anonymous=True)


    def remove_box (self, plate):
        black = np.argwhere(plate==255)
        cv2.imshow ("plate" , plate)
        cv2.waitKey(0)

        print (black)

    def get_numbers(self, plates):
        #self.remove_box(plates[0])
        
        inverted = [255 - p for p in plates]
        [cv2.imshow(str(i), inverted[i]) for i in range(len(inverted))]
        cv2.waitKey(1000)
        letters = [pytesseract.image_to_string(cv2.cvtColor(p, cv2.COLOR_GRAY2RGB)) for p in inverted]
        return (letters)

    def set_available(self, *args):
        self.available = True

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
                
                letter = thresh[y+int(h*0.10):y+int(h*0.90),x+int(w*0.05):x+int(w*0.95)]
                # cv2.imshow('letter'+str(i), letter)
                #cv2.waitKey(0)
                ret_list.append(letter)
    
    #     cv2.imshow('i', image)
    #    # cv2.waitKey(0)
        return ret_list#[x for x in ret_list if type(x) == np.ndarray and 0 not in x.shape]

    def get_image(self, data):
        return np.fromstring(data.data, dtype='uint8').reshape((data.height, data.width, 3))

    def match_pos(self, string):
        string.replace("s", "5").replace("O", "0")
        return bool(re.match("P[01]\d", string))

    def match_plate(self, string):
        return bool(re.match("[a-z|A-Z][a-z|A-Z]\d\d", string))

    def filter_results(self,strs):
        strs = [s.replace(" ", "").replace("\n", "") for s in strs]
        postision = filter(self.match_pos, strs)
        plate = filter(self.match_plate, strs)
        if len(postision)!= 1 or len(plate)!= 1:
            return None
        return postision[0], plate[0]

    def publish_plate(self, pub_str):
        rospy.loginfo(pub_str)
        self.pub.publish(pub_str)

    def callback(self, data):

        if not self.available:
            return

        image = self.get_image(data)
        cv2.imshow("image", image)
        plates = self.get_rois(image)
        [cv2.imshow(str(i), plates[i]) for i in range(len(plates))]
        print([p.shape for p in plates])
        cv2.waitKey(25)
        letters = self.get_numbers(plates)
        print(letters)
        valid_strs = self.filter_results(letters)
        if valid_strs is not None:
            self.available = False
            self.publish_plate(valid_strs[0]+" "+ valid_strs[1])
            rospy.Timer(debounce_time, self.set_available, oneshot=True)

        return letters
         
         
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
