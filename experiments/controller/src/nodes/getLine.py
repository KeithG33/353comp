#/usr/bin/env python

import cv2
import numpy as np
import sys
import math
#np.set_printoptions(threshold=sys.maxsize)

class lineFinder():

    def __init__(self,colour_cut_upper = None, colour_cut_lower = None, numpix = 200, imshape = (240,320,3)):
        if colour_cut_lower is None or colour_cut_upper is None:
            self.lower = np.array([0, 100, 65]) 
            self.upper = np.array([40, 160, 110]) 

        else:
            self.lower = colour_cut_lower
            self.upper = colour_cut_upper
        
        self.bin_cut = lambda hsv_frame : cv2.inRange(hsv_frame, self.lower, self.upper) 
        self.numpix = numpix
        self.imshape = imshape
        self.center_mass_vector = np.arange(-imshape[1]/2, imshape[1]/2, step = 1, dtype=int)
        self.ff = False

    def get_line_pos(self, frame):
    
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # if not self.ff and :
        #     print('here')
        #     with open('frame.txt', 'w+') as f:
        #         f.write(str(hsv))
        #     cv2.imwrite('frame.jpeg', hsv)
        #     self.ff = True
        #     print('done')
        #cv2.imshow('hsv',hsv)
        mask = self.bin_cut(hsv)
        cv2.imshow('mask', mask)
        cv2.waitKey(25)
        bottom = mask[:][-self.numpix:]
        # cv2.imshow('hsv', bottom)
        # if cv2.waitKey(1) == ord('q'):
        #     pass
        totalW = bottom.sum(axis=1)
        dotprod = np.dot(bottom,self.center_mass_vector)
        line = [(int(self.imshape[1]/2 + dotprod[i]/totalW[i]), self.imshape[0] - len(totalW) + i)
     for i in range(len(totalW)) if totalW[i]]
        
        #print(line)
        #for x,y in line:
        #    cv2.circle(frame, (x, y), 1, (0,0,255), -1)
        mean = np.mean([x for x,y in line])
        #mean = int(mean) if not math.isnan(mean) else None
        #cv2.circle(frame, (mean, self.imshape[1] - self.numpix/2), 10, (255,0,100), -1)
        #cv2.imshow('frame', frame)
        if math.isnan(mean):
            return "NaN"
        
        
        return mean
    def get_mean_line(self, frame):
        return np.mean([x for x,y in self.get_line_pos(frame)])
        

        
