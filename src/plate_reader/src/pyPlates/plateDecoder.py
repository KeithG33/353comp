from BoundingBoxes import boxgetter
import joblib as jb
import pickle as pkl
import numpy as np
from tensorflow import keras
import tensorflow as tf
import cv2
import os

class plateDecoder:

    def __init__(self, model=None, boxget = None, encoder = None):

        self.config = tf.ConfigProto(
        device_count={'GPU': 1},
        intra_op_parallelism_threads=1,
        allow_soft_placement=True)

        
        self.config.gpu_options.allow_growth = True
        self.config.gpu_options.per_process_gpu_memory_fraction = 0.6

        self.session = tf.Session(config=self.config)
        keras.backend.set_session(self.session)
        self.model = keras.models.load_model(filepath=model)
        

        if (boxget == None):
            self.boxget = boxgetter()

        else:
            self.boxget = boxget

        if (type(encoder) == str):
            self.enc = jb.load(encoder)

        else:
            self.enc = encoder

        self.model_shape = (64,64)#self.model.layers[0].input_shape[1], self.model.layers[0].input_shape[2]
    

    def get_nums(self, frame_arr):
        frame = [np.resize(cv2.resize(f, self.model_shape).astype(float)/255, (64,64,1)) for f in frame_arr]
        frame = np.stack(frame)
        with self.session.as_default():
            with self.session.graph.as_default():
                pred = self.model.predict(frame)
        pred = [np.argmax(f) for f in pred]
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