#usr/bin/env python
import numpy as np 
import cv2 

bin_cut = lambda hsv_frame: cv2.inRange(hsv_frame, (0, 0, 160), (255, 30, 255))
IMAGE_H, IMAGE_W = (720, 1280)
src = np.float32([[0, IMAGE_H], [IMAGE_W, IMAGE_H], [0, 0], [IMAGE_W, 0]])
dst = np.float32([[500, IMAGE_H], [IMAGE_W-500, IMAGE_H], [0, 0], [IMAGE_W, 0]])
M = cv2.getPerspectiveTransform(src, dst)  # The transformation matrix
Minv = cv2.getPerspectiveTransform(dst, src)  # Inverse transformation
minLineLength = 2
Line_thresh = 2
dx_last = 0

class curb_finder:

    def __init__(self):
        pass

    @staticmethod     
    def get_dx(line):
        return (line[2]-line[0])*(-1 if line[3]>line[1] else 1)

    @staticmethod
    def Get_Birds_Eye(Image):
        Image = Image[550:(1+IMAGE_H), 0:IMAGE_W] 
        warped_img = cv2.warpPerspective(Image, M, (IMAGE_W, IMAGE_H))  # Image warping
        hsv = cv2.cvtColor(warped_img, cv2.COLOR_BGR2HSV)
        cut = bin_cut(hsv)
        return cut, warped_img


    def get_curb_pos(self, img):
        global dx_last

    

        Birds_Eye_Road, warped = curb_finder.Get_Birds_Eye(img)

        Birds_Eye_Road = Birds_Eye_Road[0:Birds_Eye_Road.shape[0]/4, Birds_Eye_Road.shape[1]/2:]


        # cv2.imshow("birds", Birds_Eye_Road )
        # cv2.waitKey(25)

        A = Birds_Eye_Road[0:5, :]
        B = Birds_Eye_Road[-5:, :]
        M = cv2.moments(A)
    
        if M["m00"] == 0:
            dx = dx_last
        else:
            cX_A = int(M["m10"]/M["m00"])

            M = cv2.moments(B)
            cX_B = -1
            if(M["m00"] == 0):
                dx = dx_last
            else:
                cX_B = int(M["m10"]/M["m00"])

                # dx = Find_Dx(lines[index][jdex]) + 6
                dx = cX_A-cX_B + 3
        
        dx_last = dx
        print(dx)

        Left = False
        Right = False
        Drive1 = False
        if dx < -1*Line_thresh:
            return "left"
        elif dx > Line_thresh:
            return "right"
        else:
            return "straight"

