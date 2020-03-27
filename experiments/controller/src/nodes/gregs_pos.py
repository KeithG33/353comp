import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from skimage import img_as_ubyte
from geometry_msgs.msg import Twist
import math
import time
from Plate_Stuff import Check_For_Car, Plate_Read
x_Last = 0
Last_state = 0
check = True
Crosswalk_time = 5
time_1 = Crosswalk_time + 1
time_2 = 0
isCar = False
turny_curb = False
bin_cut = lambda hsv_frame: cv2.inRange(hsv_frame, (0, 0, 160), (255, 30, 255))
IMAGE_H, IMAGE_W = (720, 1280)
src = np.float32([[0, IMAGE_H], [IMAGE_W, IMAGE_H], [0, 0], [IMAGE_W, 0]])
dst = np.float32([[500, IMAGE_H], [IMAGE_W-500, IMAGE_H], [0, 0], [IMAGE_W, 0]])
M = cv2.getPerspectiveTransform(src, dst)  # The transformation matrix
Minv = cv2.getPerspectiveTransform(dst, src)  # Inverse transformation
minLineLength = 2
Line_thresh = 2
dx_last = 0
name_pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=10)


# Sends velocity commands to the skid velocity node
def Drive(speed, turn):
    name_pub = rospy.Publisher('/R1/skid_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    if not rospy.is_shutdown():
        vel_msg = Twist()
        vel_msg.linear.x = speed
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = turn
        rate.sleep()
        name_pub.publish(vel_msg)


def Drive_Forward(direction):
    global name_pub
    # name_pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    if not rospy.is_shutdown():
        vel_msg = Twist()
        vel_msg.linear.x = direction
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        
        name_pub.publish(vel_msg)
        rate.sleep()
        # Stop()


def Turn(dir):
    global name_pub
    rate = rospy.Rate(10)
    if not rospy.is_shutdown():
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = dir
       
        name_pub.publish(vel_msg)
        rate.sleep()
        # vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        
        name_pub.publish(vel_msg)
        rate.sleep()
        
        # Stop()
        # Stop()
        # Stop()


def Stop():
    global name_pub
    rate = rospy.Rate(10)
    if not rospy.is_shutdown():
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        rate.sleep()
        name_pub.publish(vel_msg)


# Checks for a pedestrian and returns true if it is safe to cross
def Safe_To_Cross(Image):
    global time_1
    global time_2
    Im = Image.copy()
    Stop()
    Is_safe = False
    Im[Im < 200] = 0
    Im = Im[300:400, 750:900]
    # cv2.imshow('im', Image)
    # cv2.waitKey(25)
    print(np.sum(Im))
    if(np.sum(Im) > 0):
        Is_safe = True

    if(Is_safe):
        Drive(0.2, 0)
        time.sleep(0.5)
        time_1 = time.time()
        time_2 = time.time()

    return Is_safe


# Parses image to check for crosswalk presence
def Crosswalk(Image):
    hsv_im = cv2.cvtColor(Image, cv2.COLOR_RGB2HSV)
    R = cv2.inRange(hsv_im, (0, 200, 10), (1, 255, 255))
    R = R[600:720, 0:1280]
    Crosswalk = False
    if np.sum(R == 255) > 0:
        Crosswalk = True

    return(Crosswalk)


# Line following code, follows the right curb
def Line_Follow(Image):
    global Last_state
    # Crop to see only small portion of curb
    crop_img = Image[500:720, 800:1280]
    # grayscale
    gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
    gray[gray < 200] = 0
    # define the kp value and the base speed value
    kp = 0.002
    vroom = 0.1
    # Calculate the center of the line based on mass
    M = cv2.moments(gray)
    cX = -1
    cY = 0
    try:
        cX = int(M["m10"]/M["m00"])
        cY = int(M["m01"]/M["m00"])
    except:
        print("ahhhhhhhh")

    if(cX != -1):
        Last_state = cX
    # calculate the error
    Center = 240
    if(cX != -1):
        x_error = Center-cX
    else:
        x_error = Center - Last_state
        vroom = 0

    turny = float(kp*x_error)

    # send velocity command
    Drive(vroom, turny)


def Get_Birds_Eye(Image):
    Image = Image[550:(1+IMAGE_H), 0:IMAGE_W] 
    # cv2.imshow('im', Image)
    # cv2.waitKey(25) # Apply np slicing for ROI crop
    warped_img = cv2.warpPerspective(Image, M, (IMAGE_W, IMAGE_H))  # Image warping
    hsv = cv2.cvtColor(warped_img, cv2.COLOR_BGR2HSV)
    # hsv = hsv[300:500, 0:1280]
    cut = bin_cut(hsv)
    # warped_gray = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
    # warped_gray[warped_gray < 200] = 0
    # warped_gray = warped_gray[400:600, 0:1280]
    return cut, warped_img


def Find_Dx(Line):
    x1 = Line[0]
    x2 = Line[2]
    y1 = Line[1]
    y2 = Line[3]
    sign = 1
    if(y2-y1) > 0:
        sign = -1
    if(y2-y1) < 0:
        sign = 1

    Dx = (x2 - x1)*sign
    return Dx


def Line_Follow_With_cmd(Image):
    global dx_last
    Birds_Eye_Road, warped = Get_Birds_Eye(Image)
    Birds_Eye_Road = Birds_Eye_Road[0:Birds_Eye_Road.shape[0]/4, Birds_Eye_Road.shape[1]/2:]
    # cv2.imshow('h', Birds_Eye_Road)
    # cv2.waitKey(25)
    # edges = cv2.Canny(Birds_Eye_Road, 50, 150, apertureSize=3)
    # lines = cv2.HoughLinesP(image=edges, rho=1, theta=np.pi/180, threshold=10, lines=np.array([]), minLineLength=minLineLength, maxLineGap=80)
    # a, b, c = lines.shape
    # max_x = 0
    # index = 0
    # for i in range(a):
    #     for j in range(b):
    #         if(lines[i][j][0] > max_x):
    #             max_x = lines[i][j][0]
    #             index = i
    #             jdex = j

    # cv2.line(warped, (lines[index][jdex][0], lines[index][jdex][1]), (lines[index][jdex][2], lines[index][jdex][3]), (0, 0, 255), 3, cv2.LINE_AA)
    A = Birds_Eye_Road[0:5, :]
    B = Birds_Eye_Road[-5:, :]
    M = cv2.moments(A)
    cX_A = -1
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

    # print(lines[index][jdex])
    # cv2.imshow('im1', Birds_Eye_Road)
    # cv2.waitKey(25)
    # cv2.imshow('im2', warped)
    # cv2.waitKey(25)
    # input()
    # print(dx)
    Left = False
    Right = False
    Drive1 = False
    if dx < -1*Line_thresh:
        Left = True
    if dx > Line_thresh:
        Right = True
    if dx < Line_thresh and dx > -1*Line_thresh:
        Drive1 = True

    if(Left):
        Drive(0, 0.15)
    if(Right):
        Drive(0, -0.15)
    if(Drive1):
        Drive(0.2, 0)


def Line_Follow_With_cmd_Take_2(Image):
    print("Sending")
    # Crop to see only small portion of curb
    crop_img = Image[360:720, 0:1280]
    # grayscale
    gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
    gray[gray < 200] = 0
    cuts = [0]*30
    h, w = gray.shape
    index = 0
    for i in range(len(cuts)):
        w0 = i*(w/30)
        w1 = (i+1)*(w/30)
        cuts[i] = gray[0:h, w0:w1]
        if np.sum(cuts[i] > 244) > 0:
            index = i

    # print(index)
    Right_thresh = 29
    Left_thresh = 23
    Right = False
    Left = False
    Straight = False
    if index >= Right_thresh:
        Right = True
    if index <= Left_thresh:
        Left = True
    if index < Right_thresh and index > Left_thresh:
        Straight = True

    if(Left):
        Turn(1)
    if(Right):
        Turn(-1)
    if(Straight):
        Drive_Forward(1)

    cv2.circle(Image, (index*w/30, Image.shape[0]-10), 10, (255, 255, 0), -1)
    cv2.imshow('g', Image)
    cv2.waitKey(25)


def Line_Follow_With_cmd_Take_3(Image):
    global x_Last
    # Crop to see only small portion of curb
    crop_img = Image[500:720, 800:1280]
    # grayscale
    gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
    gray[gray < 200] = 0
    # Gaussian blur
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    # colour thresh
    ret, thresh = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY_INV)
    M = cv2.moments(thresh)
    cX = -1
    cY = 0
    try:
        cX = int(M["m10"]/M["m00"])
        cY = int(M["m01"]/M["m00"])
    except:
        print("ahhhhhhhh")

    if(cX != -1):
        x_Last = cX

    if(cX == -1):
        cv2.circle(Image, (x_Last, cY+600), 10, (255, 255, 0), -1)
    else:
        cv2.circle(Image, (cX+640, cY+600), 10, (255, 255, 0), -1)

    center = 245
    # cv2.imshow('g', gray)
    # cv2.waitKey(25)
    # cv2.imshow("im", Image)
    # cv2.waitKey(25)
    x_thresh = 8
    error = center-cX
    if error < -1*x_thresh:
        print("Turn Left")
        Turn(1)
    if error > x_thresh:
        print("Turn_Right")
        Turn(-1)
    if error <= x_thresh and error >= -1*x_thresh:
        print("Go Straight")
        Drive_Forward(1)
    # if(cX != -1):
    #     x_error = Center-cX
    # else:
    #     x_error = Center-x_Last

    # if x_error > x_thresh:
    #     Turn(1)
    # else:
    #     if x_error < -1*x_thresh:
    #         Turn(-1)
    #     else:
    #         Drive_Forward(1)


def Line_Follow_With_cmd_Take_4(Image):
    # Crop to see only small portion of curb
    gray = cv2.cvtColor(Image, cv2.COLOR_BGR2GRAY)
    gray[gray < 200] = 0
    gray = gray[450:650, 590:610]
    if np.sum(gray > 244) > 10:
        Turn(1)
    else:
        Drive_Forward(1)
    cv2.imshow('gray', gray)
    cv2.waitKey(25)
    # cv2.imshow('im', Image)
    # cv2.waitKey(25)


def callback(data):
    global check
    global time_1
    global time_2
    global Crosswalk_time
    global isCar
    # Converts Raw image to usable np array
    y = np.frombuffer(data.data, dtype=np.uint8)
    # A = np.ndarray.astype(y, np.uint8)
    B = np.reshape(y, (data.height, data.width, 3))
    Cross = False
    # check if it's been long enough before looking for another crosswalk

    # isCar = Check_For_Car(B)

    time_1 = time.time()
    if (time_1 - time_2) > Crosswalk_time:
        Cross = Crosswalk(B)

    if(Cross) or (check is False):
        Stop()
        print("Crosswalk")
        check = Safe_To_Cross(B)

    if(check):
        Line_Follow_With_cmd(B)


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    sub1 = rospy.Subscriber("/R1/pi_camera/image_raw", Image, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if _name_ == '_main_':
    listener()