#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import cv2
import numpy as np
from getLine import lineFinder
from curbFinder import curb_finder
line_center = 320

finder = lineFinder(imshape = (200, 640, 3) , colour_cut_lower=(0,0,160), colour_cut_upper=(255,30,255))
image_topic =  '/R1/pi_camera/image_raw'
c = curb_finder()
# look_up = {'left' : (0,1), 'right' : (0,-1), 'straight' : (1,0)}
look_up = {'left' : (0,0.3), 'right' : (0,-0.3), 'straight' : (0.4,0)}
#rospy.Subscriber('control_effort', Float64, controlEffort)
pub = rospy.Publisher('R1/skid_vel', Twist, queue_size=10)


def getCenter(image):
    global xLast

    cX = finder.get_line_pos(image)
    if (cX != 'NaN'):
        state(cX)
    else:
        state(xLast)

    return cX


def callback(data):
    global xLast
    global derr
    
    global h,w
    h,w = data.height, data.width

    # Find image matrix
    img = np.fromstring(data.data, dtype='uint8').reshape((h, w, 3))
    direc = c.get_curb_pos(img)

    talker(direc)

def listener():
	# Initialize this ros node
	rospy.init_node('vidFeed', anonymous=True)

	# This node subscribes to the camera image feed
	rospy.Subscriber(image_topic, Image, callback)
	rospy.spin()

def setpoint():
	#This bit of code continuosly sends our desired state to be 
	#middle of the screen
	setpoint_pub = rospy.Publisher('/setpoint', Float64, queue_size=10)
	setpoint = Float64()
	setpoint.data = 400.0
	setpoint_pub.publish(setpoint)

def state(x):
	#This code will send the current state to pid
	state_pub = rospy.Publisher('/state', Float64, queue_size=10)
	state = Float64()
	state.data = x
	state_pub.publish(state)

def controlEffort(val):
    global effort
    effort = val.data
    #print(effort)


def talker(direc):
  # 10hz
    
    rate = rospy.Rate(10)
    vals = look_up[direc]
	
    if not rospy.is_shutdown():
        vel_msg = Twist()
        vel_msg.linear.x = vals[0]
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = vals[1]
        pub.publish(vel_msg)
        rate.sleep()
	    
        # pass


if __name__ == '__main__':
    xLast = 0
    effort = 1
    derr = 0
    side = 1
    listener()