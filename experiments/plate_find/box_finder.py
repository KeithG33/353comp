import cv2
import numpy as np
import pytesseract
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

image_topic =  '/rrbot/camera1/image_raw'


class boxgetter():

    def __init__(self, kernel_size = (10,1), cut_max = (200,200,50), cut_min=(0, 0,0), **kwargs):
        self.kernel = np.ones(kernel_size, np.uint8)
        self.cut_max = cut_max
        self.cut_min = cut_min
        rospy.init_node('vidFeed', anonymous=True)


    def remove_box (self, plate):
        black = np.argwhere(plate==255)
        cv2.imshow ("plate" , plate)
        cv2.waitKey(0)

        print (black)

    def get_numbers(self, plates):
        #self.remove_box(plates[0])
        inverted = [255 - p for p in plates]
        letters = [pytesseract.image_to_string(cv2.cvtColor(p, cv2.COLOR_GRAY2RGB)) for p in inverted]
        return (letters)

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
        
            if w > 15 and h > 15: #more checks here
                #ret_list.append(roi)

                letter = thresh[y+5:y+h-5,x+5:x+w-5]
                # cv2.imshow('letter'+str(i), letter)
                #cv2.waitKey(0)
                ret_list.append(letter)
    
    #     cv2.imshow('i', image)
    #    # cv2.waitKey(0)
        return ret_list#[x for x in ret_list if type(x) == np.ndarray and 0 not in x.shape]

    def get_image(self, data):
        return np.fromstring(data.data, dtype='uint8').reshape((data.height, data.width, 3))


    def callback(self, data):
        print("callback")
        image = self.get_image(data)
        plates = self.get_rois(image)
        letters = self.get_numbers(plates)
        print(letters)
        return letters
         
         
    def start(self):
        print("starting")
        rospy.Subscriber(image_topic, Image, self.callback)
        rospy.spin()

if __name__ == "__main__":
    b = boxgetter()
    b.start()

    # image = cv2.imread("image2.png")
    
    
    # plates = b.get_rois(image)

    # letters = b.get_numbers(plates)
    # print(letters)
