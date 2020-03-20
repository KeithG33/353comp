from BoundingBoxes import boxgetter
import joblib as jb
import numpy as np
import cv2
import os

class plateDecoder:

    def __init__(self, model=None, model_path = None, boxget = None, encoder = None):
        if (model == None):
            if (model_path != None):
                self.model = jb.load(model_path)
        
        if (boxget == None):
            self.boxget = boxgetter()

        else:
            self.boxget = boxget

        if (type(encoder) == str):
            self.enc = jb.load(encoder)

        else:
            self.enc = encoder

        self.model_shape = self.model.layers[0].input_shape[1], self.model.layers[0].input_shape[2]
    

    def get_nums(self, frame_arr):
        frame = [cv2.resize(f, self.model_shape).astype(float)/255 for f in frame_arr]
        frame = np.stack(frame)
        pred = [np.argmax(f) for f in self.model.predict(frame)]
        return self.enc.inverse_transform(pred)

    def get_rois(self, frame):
        return self.boxget.get_letters(frame)

    def getPlate(self, frame):
        return "".join(self.get_nums(self.get_rois(frame)))


if (__name__ == '__main__'):
    x = plateDecoder(encoder = 'encoder.jb', model_path='cnn2.jb')
    # framein = cv2.imread('trainimg/0.png')
    # out = x.get_nums([framein])
    # print(out)
    path = './pictures/'
    right = []
    for f in os.listdir(path):
        framein = cv2.imread(path +  f)
        out = x.getPlate(framein)
        right.append(out == f[6:10])
        print(f, out, out == f[6:10])
    
    print('acc: ' + str((np.sum(right))/len(right)))