import numpy as np
import cv2
import os

class boxgetter():

    def __init__(self, kernel_size = (10,1), cut_max = 255, cut_min=244, **kwargs):
        self.kernel = np.ones(kernel_size, np.uint8)
        self.cut_max = cut_max
        self.cut_min = cut_min

    def get_letters(self, image):
        ret_list = []
        colour_cut = cv2.inRange(image, self.cut_min,self.cut_max)
        #cv2.imshow('colourcut', colour_cut)
        thresh = colour_cut

        img_dilation = cv2.dilate(thresh, self.kernel, iterations=1)
        #cv2.imshow('dilated', img_dilation)


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
            roi = image[y:y + h, x:x + w]

            # show ROI
            # cv2.imshow('segment no:'+str(i),roi)
            #cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
            if w > 15 and h > 15:
                ret_list.append(roi)
            #    letter = image[y:x+w,y:y+h]
            #    ret_list.append(letter)
    
        return ret_list#[x for x in ret_list if type(x) == np.ndarray and 0 not in x.shape]

if __name__ == '__main__':
    numletter = dict()
    x = boxgetter()
    path = 'Liscence_plates/Liscence_Plate_nn/pictures/'
    for F in os.listdir(path):
        print(F)
        img_list = x.get_letters(cv2.imread(path+F))
        count = 0
        for i in F[6:10]:
            if i in numletter.keys():
                numletter[i] += 1
                filename = str(i) + str(numletter[i])
            else: 
                numletter[i] = 0
                filename = str(i)
            
            cv2.imwrite('trainimg/{}.png'.format(filename), img_list[count])
            count +=  1

