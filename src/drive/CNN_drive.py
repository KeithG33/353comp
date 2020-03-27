import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import roslib
from keras import models
from keras import backend

model = models.load_model("/home/fizzer/353comp/src/competition_2019t2/nodes/driver_cnn.h5")
model._make_predict_function()

ANGULAR_SPEED = 0.075
LINEAR_SPEED = 0.065

class CNN_driver():

    def __init__(self):
        self.move = Twist()
        self.move_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,self.callback)


    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        img_normed = cv_image / 255
        processed_img = np.expand_dims(img_normed,axis=0)

        y_predicts = model.predict(processed_img)
        prediction = y_predicts.argmax(axis=1)[0]
        prediction = int(prediction)

        if prediction == 0:
            print("Going forward...")
            self.move.linear.x = LINEAR_SPEED
            self.move.angular.z = 0   
        elif prediction == 1:
            print("Going backward...")
            self.move.linear.x = -LINEAR_SPEED
            self.move.angular.z = 0  
        elif prediction == 2:
            print("Rotating left...")
            self.move.linear.x = 0
            self.move.angular.z = ANGULAR_SPEED
        elif prediction == 3:
            print("Rotating right...")
            self.move.linear.x = 0
            self.move.angular.z = -ANGULAR_SPEED
            
        self.move_pub.publish(self.move)

def main(args):
    driver = CNN_driver()
    rospy.init_node('cnn_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
    

if __name__ == '__main__':
    main(sys.argv)