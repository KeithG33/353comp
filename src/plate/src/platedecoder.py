#!/usr/bin/env python
import cv2
import numpy as np
import Queue
import pytesseract
import re
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Int32
from cv_bridge import CvBridge, CvBridgeError

image_topic =  'extracted_plates'
sync_topic = "sync"


class plate_decoder():

    def __init__(self, **kwargs):
        rospy.init_node('plate_decoder', anonymous=True)
        self.bridge = CvBridge()
        self.num_exp = Queue.Queue()
        self.good_pos = None
        self.good_plate = None
        self.num_computed = 0
        self.finished = False
        self.pub = rospy.Publisher('plate', String, queue_size=10)

    def get_numbers(self, plate):

        # [cv2.imshow(str(i), plates[i]) for i in range(len(plates))]
        # cv2.waitKey(1000)
        return pytesseract.image_to_string(plate)


    def get_image(self, data):
        return self.bridge.imgmsg_to_cv2(data, "rgb8")

    def match_pos(self, string):
        new = string.replace("S", "5").replace("O", "0").replace("i","1").replace("Q","0")
        print(new)
        return bool(re.match("P[01]\d", new))

    def match_plate(self, string):
        return bool(re.match("[a-z|A-Z][a-z|A-Z]\d\d", string))

    def filter_results(self,s):
        s = s.upper()
        s = re.sub(r'\W+', '', s)
        print (s)
        postision = self.match_pos(s)
        plate = self.match_plate(s)
        if postision  == True:
            self.good_pos = s.replace("S", "5").replace("O", "0").replace("i","1").replace("Q","0")
        elif plate == True:
            self.good_plate = s
        else:
            return False
        return True

    def publish_plate(self, pub_str):
        rospy.loginfo(pub_str)
        self.pub.publish(pub_str)

    def set_sync(self, msg):
        self.num_exp.put(msg.data)

    def callback(self, data):
        self.num_computed+=1
        if not self.finished:
            image = self.get_image(data)
            # cv2.imshow(str(self.num_computed), image)
            # cv2.waitKey(0)
            reading = self.get_numbers(image)
            #print(reading)
            valid_strs = self.filter_results(reading)
         
            if self.good_pos != None and self.good_plate != None:
                print
                self.publish_plate(self.good_pos+" "+ self.good_plate)
                self.good_plate = None
                self.good_pos = None
                self.finished = True

        if self.num_computed == 2:
            self.num_computed = 0
            self.finished = False
        

         
         

    def start(self):
        rospy.Subscriber(sync_topic, Int32, self.set_sync,queue_size=3)
        rospy.Subscriber(image_topic, Image, self.callback, queue_size=6)
        rospy.spin()

if __name__ == "__main__":
    b = plate_decoder()
    b.start()

    # image = cv2.imread("image2.png")
    
    
    # plates = b.get_rois(image)

    # letters = b.get_numbers(plates)
    # print(letters)
