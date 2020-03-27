import cv2
import numpy as np

colour_cut_lower=(0, 100,40)
colour_cut_upper=(30,255,255)



def find_plate(img):
    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, colour_cut_lower, colour_cut_upper)
    pix_y, pix_x = np.where(mask==255) 

    if (len(pix_y)==0):
        return None

    mask[:mask.shape[0]/2,:] = 0 
    mask [:pix_y[0]+20,:] =0

    #cv2.circle(img, (pix_x[0], pix_y[0] ), 5 ,(0,255,0), 2)

    im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


    if (len(contours)<2):
        return None

    list.sort(contours, key = lambda cnt : cv2.contourArea(cnt), reverse=  True)

    cv2.drawContours(img, contours[:2], -1, (0,255,0), 3)
    big_x,big_y,big_w,big_h = cv2.boundingRect(contours[0])
    lil_x,lil_y, lil_w, lil_h = cv2.boundingRect(contours[1])
   
    

    plate = img[lil_y:big_y+big_h,lil_x+lil_w:big_x ,:]
    cv2.rectangle(img,(lil_x+lil_w,lil_y),(big_x, big_y+big_h) ,(0,255,0),2)

    cv2.imshow("mask", mask)
    cv2.waitKey(25)

    return plate