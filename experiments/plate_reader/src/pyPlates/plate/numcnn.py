import keras as k
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import cv2 as cv

from sklearn.ensemble import RandomForestClassifier

#Create a Gaussian Classifier
clf=RandomForestClassifier(n_estimators=100)

img_files = os.listdir("../Edited_plates")
data = pd.DataFrame()
data['Path'] = img_files
data['Value'] = [F[0] for F in img_files]
data['frame'] = [np.array(cv.imread("Edited_plates/" + F)) for F in img_files]