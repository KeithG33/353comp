#!/usr/bin/env python
from detec_pd import no_vehicular_manslaughter
from detec_cw import saftey_first 

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class pd_dector:

    def __init__(self):
        self.stop_pub = rospy.Publisher('stop', String, queue_size=10)
        rospy.init_node('stop', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,self.callback)
        self.rate = rospy.Rate(10) # 10hz
        self.last_sent_go = False
        rospy.spin()

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if saftey_first.is_cw(cv_image) and no_vehicular_manslaughter.is_pd(cv_image):
            self.pub_msg()
            self.last_sent_go = False
        
        elif self.last_sent_go == False: 
            self.pub_msg("go")
            self.last_sent_go = True


    def pub_msg(self, val="stop"):
        rospy.loginfo(val)
        self.stop_pub.publish(val)
        self.rate.sleep()

def main(*args):
    detector = pd_dector()


if __name__ == "__main__":
    main()