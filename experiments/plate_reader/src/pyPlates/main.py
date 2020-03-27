import rospy
from keras import models
from sensor_msgs.msg import Image
import cv2
import numpy as np
from plateFinder import find_plate
from plateDecoder import plateDecoder

model = 'pyPlates/model.h5'

enc = 'pyPlates/encoder.jb'

decoder = plateDecoder(model = model, encoder=enc)

image_topic = '/R1/pi_camera/image_raw'

def callback(data):

    # Find image matrix
    img = np.fromstring(data.data, dtype='uint8').reshape((data.height, data.width, 3))
    
    plate = find_plate(img)

    if plate is not None and 0 not in plate.shape:
        
        letters = decoder.get_rois(plate)
        cv2.imshow('plate', plate)
        if (len(letters)==4):
            print("predicting")
            pred = decoder.get_nums(letters)
            plate_str = "".join(pred)
            print(plate_str)


    else:
        pass
        #print("no plate")
    cv2.imshow('img', img)



    cv2.waitKey(25)

    # if plate is not None:
    #     talker("FF69", 420)

def listener():
	rospy.init_node('vidFeed', anonymous=True)
	rospy.Subscriber(image_topic, Image, callback)
	rospy.spin()

def talker(plate_str, plate_num):
    pass


def main():
    listener()

if __name__ == "__main__":
    main()