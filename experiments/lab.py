#!/usr/bin/env python3
import anki_vector as av
from sys import argv
import threading
import cv2
import numpy as np
from anki_vector import events
import time
import anki_vector
from anki_vector.util import distance_mm, speed_mmps, degrees

class sifter ():
    
    def __init__(self):
        self.make_sift()


    def make_sift(self):
        # queryiamge
        self.img = cv2.imread("block_pattern.jpg", cv2.IMREAD_GRAYSCALE)
        self.sift = cv2.xfeatures2d.SIFT_create()
        self.kp_image, self.desc_image = self.sift.detectAndCompute(self.img, None)
        # Feature matching
        self.index_params = dict(algorithm=0, trees=5)
        self.search_params = dict()
        self.flann = cv2.FlannBasedMatcher(self.index_params, self.search_params)
    


    def do_sift(self, robot,  event_type, event, done):
        img_pil = robot.camera.latest_image.raw_image
        frame = cv2.cvtColor(np.array(img_pil), cv2.COLOR_BGR2GRAY)

        kp_grayframe, desc_grayframe = self.sift.detectAndCompute(frame, None)
        matches = self.flann.knnMatch(self.desc_image, desc_grayframe, k=2)
        good_points = []
        for m, n in matches:
            if m.distance < 0.6 * n.distance:
                good_points.append(m)


        
        query_pts = np.float32([self.kp_image[m.queryIdx].pt for m in good_points]).reshape(-1, 1, 2)
        train_pts = np.float32([kp_grayframe[m.trainIdx].pt for m in good_points]).reshape(-1, 1, 2)

        failed = False

        if (len (query_pts) == 0 or len(train_pts) == 0):
            failed = True
            print("Failed to find points")

        else:         
            matrix, mask = cv2.findHomography(query_pts, train_pts, cv2.RANSAC, 5.0)
            matches_mask = mask.ravel().tolist()
           
    
        h,w  = self.img.shape
        pts = np.float32([[0, 0], [0, h], [w, h], [w, 0]]).reshape(-1, 1, 2)

        if  failed or matrix is None :
            print("nothing")
            homography = frame
            x = None
        else:
            dst = cv2.perspectiveTransform(pts, matrix)
            x = np.mean(dst[:,0,0])
            print(x)
            homography = cv2.polylines(frame, [np.int32(dst)], True, (255, 0, 0), 3)
            
            if (x<200):
                robot.behavior.turn_in_place(degrees(5))
        
            elif (x>450):
                robot.behavior.turn_in_place(degrees(-5))

            else:
                robot.behavior.drive_straight(distance_mm(25), speed_mmps(100)) 

        
        cv2.imshow("Homography", homography)
        cv2.waitKey(1)
    #event.image.show()
    #done.set()



def main():
    # Modify the SN to match your robot’s SN
    ANKI_SERIAL = '00302fc4'
    ANKI_BEHAVIOR = av.connection.ControlPriorityLevel.OVERRIDE_BEHAVIORS_PRIORITY

    with av.Robot(serial=ANKI_SERIAL,
                  behavior_control_level=ANKI_BEHAVIOR, show_viewer=True) as robot:

        #time.sleep(5)
        robot.world.connect_cube()
        robot.camera.init_camera_feed()
        #image = robot.camera.latest_image
        #image.raw_image.show()
        s = sifter()
        done = threading.Event()
        robot.events.subscribe(s.do_sift, events.Events.new_raw_camera_image, done)
        # robot.camera.init_camera_feed()
        # image = robot.camera.latest_image
        # image.raw_image.show()

        try:
            while not done.wait(timeout=10):
                #image = robot.camera.latest_image
                #image.raw_image.show()
                print("------ Did not receive a new camera image! ------")
        except KeyboardInterrupt:
            quit()

if __name__ == "__main__":
    main()