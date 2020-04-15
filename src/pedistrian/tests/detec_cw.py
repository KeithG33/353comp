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
        cv2.imshow("mask", mask)
        cv2.waitKey(1000)

        if (np.sum(mask*1.0/255)<8000):
            return False


        if ret_x_pos == True:
            x_min = np.min(np.where(mask==255))
            x_max = np.max(np.where(mask==255))
            return True, x_min, x_max
        
        return True


def test_detection(my_func, test_img_path):

    true_negatives = 0
    img_dir = test_img_path+"/no/"
    for path in os.listdir(img_dir):
        img = cv2.imread(img_dir+ path)
        if (my_func(img)==False):
            true_negatives += 1
            print (path + ": correct")

        else:
            print (path + ": incorrect")

    print (str(true_negatives)  + " out of " + str(len(os.listdir(test_img_path+"/no"))) + " negatives correct")

    img_dir = test_img_path+"/yes/"
    true_positivies = 0
    for path in os.listdir(img_dir):
        img = cv2.imread(img_dir + path)
        if (my_func(img)==True):
            true_positivies += 1
            print (path + ": correct")

        else:
            print (path + ": incorrect")

    print (str(true_positivies)  + " out of " + str(len(os.listdir(test_img_path+"/yes"))) + " postives correct")


if __name__ == "__main__":
    test1 = cv2.imread(test_img_path+'/yes/1.png')
    saftey_first.is_cw(test1, True)
    test_detection(saftey_first.is_cw, test_img_path)