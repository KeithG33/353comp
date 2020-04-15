import cv2 
import numpy as np 
import os
test_img_path = "testimgs/ped/"


class saftey_first:


    @staticmethod
    def is_cw(img, ret_x_pos=False):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0,200,100])
        upper_red = np.array([20,255,255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        # cv2.imshow("mask", mask)
        # cv2.waitKey(1000)

        if (np.sum(mask*1.0/255)<8000):
            return False


        if ret_x_pos == True:
            x_min = np.min(np.where(mask==255))
            x_max = np.max(np.where(mask==255))
            return True, x_min, x_max
        
        return True


