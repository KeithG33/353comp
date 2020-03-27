#usr/bin/env python
import numpy as np 
import cv2 
import matplotlib.pyplot as plt
import imutils

CROP_FRAC = 1/3
bin_cut = lambda hsv_frame : cv2.inRange(hsv_frame, (0,0,160), (255,30,255))
str8thresh = 8

class curb_finder:

    def __init__(self):
        pass

    @staticmethod     
    def get_dx(line):
        return (line[2]-line[0])*(-1 if line[3]>line[1] else 1)

    def get_curb_pos(self, img):
        
        imshape = img.shape
        IMAGE_H, IMAGE_W = (720, 1280)
        src = np.float32([[0, IMAGE_H], [IMAGE_W, IMAGE_H], [0, 0], [IMAGE_W, 0]])
        dst = np.float32([[500, IMAGE_H], [IMAGE_W-500, IMAGE_H], [0, 0], [IMAGE_W, 0]])
        M = cv2.getPerspectiveTransform(src, dst)  # The transformation matrix
        Minv = cv2.getPerspectiveTransform(dst, src)  # Inverse transformation
        minLineLength = 2
        Line_thresh = 2
        dx_last = 0

        
        IMAGE_H , IMAGE_W , _ = img.shape

        src = np.float32([[0, IMAGE_H], [1207, IMAGE_H], [0, 0], [IMAGE_W, 0]])
        dst = np.float32([[569, IMAGE_H], [711, IMAGE_H], [0, 0], [IMAGE_W, 0]])
        M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
        Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation

        #img = cut # Read the test img
        img = img[450:(450+IMAGE_H), 0:IMAGE_W] # Apply np slicing for ROI crop
        warped_img = cv2.warpPerspective(img, M, (IMAGE_W, IMAGE_H)) # Image warping
        hsv = cv2.cvtColor(warped_img, cv2.COLOR_BGR2HSV)
        #cv2.imshow('transfrom', cv2.cvtColor(warped_img, cv2.COLOR_BGR2RGB)) # Show results
        hsv = hsv[300:600,:]
        cut = bin_cut(hsv)
        cv2.imshow('cut', cut) # Show results
        cv2.waitKey(25)
        edges = cv2.Canny(cut, 50, 150, apertureSize=3)
        minLineLength = 10
        lines = cv2.HoughLinesP(image=edges, rho=1, theta=np.pi/180, threshold=100, lines=np.array([]), minLineLength=minLineLength, maxLineGap=80)
        a, b, c = lines.shape
        max_x = 0
        index = 0
       
        for i in range(a):
            for j in range(b):
                if(lines[i][j][0] > max_x):
                    max_x = lines[i][j][0]
                    index = i
                    jdex = j



        #cv2.line(warped_img, (lines[index][jdex][0], lines[index][jdex][1]), (lines[index][jdex][2], lines[index][jdex][3]), (0, 0, 255), 3, cv2.LINE_AA)
        dx = curb_finder.get_dx(lines[index][jdex])
        print(dx)
        if abs(dx) < str8thresh:
            return "straight"

        elif dx < 0:
            return "left"

        else:
            return "right"

        # _, contours,hier = cv2.findContours(cut,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        # cnt = contours[0]
        # # then apply fitline() function
        # [vx,vy,x,y] = cv2.fitLine(cnt,cv2.DIST_L2,0,0.01,0.01)

        # # Now find two extreme points on the line to draw line
        # lefty = int((-x*vy/vx) + y)
        # righty = int(((cut.shape[1]-x)*vy/vx)+y)

        # #Finally draw the line
        # cv2.line(warped_img,(cut.shape[1]-1,righty),(0,lefty),255,2)
        # cv2.imshow('img',warped_img)
        # cv2.waitKey(25)
        #cv2.destroyAllWindows()

        # for c in cnts:
        #     # compute the center of the contour
        #     M = cv2.moments(c)
        #     cX = int(M["m10"] / M["m00"])
        #     cY = int(M["m01"] / M["m00"])
        #     centers.append((cX,cY))
        
        #     # draw the contour and center of the shape on the contour_img
        #     cv2.drawContours(contour_img, [c], -1, (0, 255, 0), 2)
        #     cv2.circle(contour_img, (cX, cY), 7, (255, 255, 255), -1)
        #     cv2.putText(contour_img, "center", (cX - 20, cY - 20),
        #         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        #     break
        #     # show the contour_img


        # # print(inner[0][0][0])
        # cv2.drawContours(contour_img, contours, -1, (0,255,0), 3)
        # # cv2.circle(contour_img, (inner[0][0][0][0], inner[0][0][0][1]), 3, (0,0,255))
        # # cv2.drawContours(contour_img, contours, 1, (255,0,0))
        # # for index, val in np.ndenumerate(peaks[:,:,0]):
        # #     if (val!=0):
        # #         cv2.circle(cut, index, 1, 255)
        # # print(peaks.shape)
        # # cv2.imshow('contours', contour_img)
        # # cv2.imshow('i',img)
        # # cv2.imshow('cut',cut)
        # cv2.waitKey(25)
        # print(cX, cY, imshape[0], imshape[1])
        # print(len(contours))
        # return "left"
        # if (cX<imshape[1]*1/3 or cY > imshape[0]*5/6):
        #     print('l')
        #     return "left"

        # if(cX>imshape[1]*5/6):
        #     print('r')
        #     return "right"

        # else:
        #     print('s')
        #     return "straight"
        return 'straight'

def simple_line_follower(img):

    