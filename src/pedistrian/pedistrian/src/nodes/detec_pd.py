import cv2
import numpy as np 
import os
import detec_cw


test_path = "testimgs/ped/"
test_yes = test_path + "yes/"
test_no = test_path + "no/"

class no_vehicular_manslaughter:

    lower_red = np.array([0,2,0])
    upper_red = np.array([20,200,50])

    @staticmethod
    def get_mean_head_pos(mask):
        return 0.5*(np.min(np.where(mask==255))+np.min(np.where(mask==255)))

    @staticmethod
    def make_mask(img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        return cv2.inRange(hsv, no_vehicular_manslaughter.lower_red,  no_vehicular_manslaughter.upper_red)
    @staticmethod 
    def is_pd(img):
        mask = no_vehicular_manslaughter.make_mask(img)
        # cv2.imshow("mask", mask)
        # cv2.waitKey(1000)


        return np.sum(mask*1.0/255)>10 and no_vehicular_manslaughter.get_mean_head_pos(mask) > detec_cw.saftey_first.is_cw(img, True)[1]



if __name__ == "__main__":
    from detec_cw import test_detection as test
    # img = cv2.imread(test_yes+"1.png")
    # print(no_vehicular_manslaughter.detect_pd(img))
    test(no_vehicular_manslaughter.is_pd, test_path)

