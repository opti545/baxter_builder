#!/usr/bin/env python
#The structure of this file was taken from the baxter_stocking_stuffer project by students in Northwestern's MSR program - Josh Marino, Sabeen Admani, Andrew Turchina and Chu-Chu Igbokwe
# Message published - opencv/center_of_object - contains x,y,z coordinates as a Point message

import rospy
import numpy as np
import cv2
import baxter_interface

from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from cv_bridge import CvBridge, CvBridgeError

from baxter_builder.srv import *


#green
low_h  = 60
high_h = 90
low_s  = 85
high_s = 175
low_v  = 70
high_v = 255
'''
#white
low_h=0
high_h=0
low_s=0
high_s=0
low_v=0
high_v=255
'''
#blue
#low_h  = 105
#high_h = 115
#low_s  = 135
#high_s = 160
#low_v  = 20
#high_v = 60

xpos = 0
ypos = 0
zpos = 0


#Thresholds image and stores position of object in (x,y) coordinates of the camera's frame, with origin at center.
def callback(message):

    #Capturing image of web camera
    bridge = CvBridge()

    cv_image = bridge.imgmsg_to_cv2(message, "bgr8")
    height, width, depth = cv_image.shape
    #print height, width, depth 


    #Converting image to HSV format
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    thresholded = cv2.inRange(hsv, np.array([low_h, low_s, low_v]), np.array([high_h, high_s, high_v]))


    #Morphological opening (remove small objects from the foreground)
    thresholded = cv2.erode(thresholded, np.ones((2,2), np.uint8), iterations=1)
    thresholded = cv2.dilate(thresholded, np.ones((2,2), np.uint8), iterations=1)

    #Morphological closing (fill small holes in the foreground)
    thresholded = cv2.dilate(thresholded, np.ones((2,2), np.uint8), iterations=1)
    thresholded = cv2.erode(thresholded, np.ones((2,2), np.uint8), iterations=1)


    ##Finding center of object
    ret,thresh = cv2.threshold(thresholded,157,255,0)
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

    M = cv2.moments(thresh)
    
    A = M['m10']
    B = M['m00']

    if B>500:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        P = Point()
        # .0025 is pixel size at 1 meter
        # .266 is the vertical distance from camera to object
        # .7 and .5 is the position of the gripper in baxter's coordinates
        # .05 and -.02 is camera offset from gripper
        #P.x = (cx -(width)/2)*.0025*.3 + .7 + .05 #+.1
        #P.y = (cy - (height)/2)*.0025*.3 + .5 - .02 #- .1
        #dist = baxter_interface.analog_io.AnalogIO('left_hand_range').state()
        #print "Distance is %f" % dist
        #dist = dist/1000
        P.x = (cx - (width/2))*.0025*.623 + .603 + .05 #- .1
        P.y = (cy - (height/2))*.0025*.414 + .435  - .02 #+.1

        pub.publish(P)


    #Printing to screen the images
    cv2.imshow("Original", cv_image)
    cv2.imshow("Thresholded", thresholded)
    cv2.waitKey(3)


def get_obj_location(request):

    response = 1
    obj_found = False
    correct_location = False

    if request.startobjloc == True:
       # get camera frame
       
       #scan for objects

       if obj_found == False:
           print "No objects were found in the current camera frame. "
           response.objfound = False
           response.zready = False
           response.xb = 0.0
           response.yb = 0.0
           response.zb = 0.0
       else:
           response.objfound = True
           print "Object(s) were found in the current camera frame. "
           #Find (x,y,z) coordinate of the centroid of the object
           response.xb = xpos
           response.yb = ypos
           response.zb = zpos

           #Check if centroid location is close enough to center of frame 
           if correct_location == True:
               esponse.zready = True
               print "Location of the object's centroid is at the center of the camera frame. "
               print "Ready for grasping. "
           else:
               esponse.zready = False
               print "Not ready for grasping. "

    return response

'''
Creates a service that provides information about the loaction of an object.
Subscribes to left hand camera image feed
'''
def main():

    #Initiate left hand camera object detection node
    rospy.init_node('left_camera_node')

    #Create names for OpenCV images and orient them appropriately
    cv2.namedWindow("Original", 1)
    cv2.namedWindow("Thresholded", 2)

    #Subscribe to left hand camera image 
    rospy.Subscriber("/cameras/left_hand_camera/image", Image, self.callback)

    #Declare object location service called object_location_service with ObjLocation service type.
    #All requests are passed to get_obj_location function
    obj_location_srv = rospy.Service("object_location_service", ObjLocation, self.get_obj_location)

    print "Left - Ready to find object."

    #Keeps code from exiting until service is shutdown
    rospy.spin()


if __name__ == '__main__':
     main()