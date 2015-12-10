#!/usr/bin/env python
#Reference: the baxter_stocking_stuffer project by students in Northwestern's MSR program - Josh Marino, Sabeen Admani, Andrew Turchina and Chu-Chu Igbokwe
#Service provided - ObjLocation service - contains x,y,z coordinates of object in baxter's stationary body frame, whether it is ok to grasp and if objects were found in the current frame.

import rospy
import numpy as np
import cv2
import cv_bridge
import baxter_interface

from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from cv_bridge import CvBridge, CvBridgeError

from baxter_builder.srv import *

'''
#green
g_low_h  = 60
g_high_h = 90
g_low_s  = 85
g_high_s = 175
g_low_v  = 70
g_high_v = 255

#white
w_low_h=0
w_high_h=0
w_low_s=0
w_high_s=0
w_low_v=0
w_high_v=255

#blue
b_low_h  = 105
b_high_h = 115
b_low_s  = 135
b_high_s = 160
b_low_v  = 20
b_high_v = 60
'''

# global obj_found
obj_found = False
# global correct_location 
correct_location = True
#Object color: 0 = green, 1 = blue
obj_color = 0

#Object centroid position in the baxter's stationary base frame 
xb = 0
yb = 0


'''
Thresholds camera image and stores object centroid location (x,y) in Baxter's base frame.
'''
def callback(message):

    global xb, yb, obj_color, obj_found
    xb = 0
    yb = 0
    #Capturing image of camera
    bridge = CvBridge()

    cv_image = bridge.imgmsg_to_cv2(message, "bgr8")
    height, width, depth = cv_image.shape
    #print height, width, depth 

    #Converting image to HSV format
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    thresholded = 0

    obj_color = 0

    #Green colored objects
    if obj_color == 0: 
        # Analyze image for green objects
        low_h  = 60
        high_h = 90
        low_s  = 85
        high_s = 175
        low_v  = 70
        high_v = 255

        thresholded = cv2.inRange(hsv, np.array([low_h, low_s, low_v]), np.array([high_h, high_s, high_v]))

        #Morphological opening (remove small objects from the foreground)
        thresholded = cv2.erode(thresholded, np.ones((2,2), np.uint8), iterations=1)
        thresholded = cv2.dilate(thresholded, np.ones((2,2), np.uint8), iterations=1)

        #Morphological closing (fill small holes in the foreground)
        thresholded = cv2.dilate(thresholded, np.ones((2,2), np.uint8), iterations=1)
        thresholded = cv2.erode(thresholded, np.ones((2,2), np.uint8), iterations=1)

        ##Finding center of object
        #cv2.threshold(source image in greyscale, threshold value which is used to classify the pixel values, the maxVal which represents the value to be given if pixel value is more than (sometimes less than) the threshold value )
        #output: retval and thresholded image
        ret,thresh = cv2.threshold(thresholded,157,255,0)

        #findCountours(source image, contour retrieval mode, contour approximation method )
        #contours is a Python list of all the contours in the image. Array of arrays.
        #Each individual contour is a Numpy array of (x,y) coordinates of boundary points of the object.
        #hierarchy is holding information on the nesting of contours
        contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        #can also use cv2.CHAIN_APPROX_SIMPLE that removes all redundant points and compresses the contour, thereby saving memory.

        #Draw the countours. 
        #drawCountours(source image, contours as a Python list, index of contours (useful when drawing individual contour. To draw all contours, pass -1),color, thickness)
        cv2.drawContours(cv_image, contours, -1, (0,255,0), 3)

        numobj = len(contours) # number of objects found in current frame
        #print 'Number of objects found in the current frame: ' , numobj

        if numobj > 0:
            moms = cv2.moments(contours[0])
            if moms['m00']>500:
                cx = int(moms['m10']/moms['m00'])
                cy = int(moms['m01']/moms['m00'])
                
                # print 'cx = ', cx
                # print 'cy = ', cy

                #dist = baxter_interface.analog_io.AnalogIO('left_hand_range').state()
                #print "Distance is %f" % dist
                #dist = dist/1000

                #Position in the baxter's stationary base frame
                xb = (cy - (height/2))*.0023*.433 + .712 + .02
                yb = (cx - (width/2))*.0023*.433 + .316  - .02
            #print "Found green ", numobj,  "object(s)"
            obj_found = True
        else:
            obj_color = 1 #No green objects were found so switch to red
            # Analyze image for red objects
            low_h  = 0
            high_h = 3
            low_s  = 130
            high_s = 190
            low_v  = 80
            high_v = 250

            thresholded = cv2.inRange(hsv, np.array([low_h, low_s, low_v]), np.array([high_h, high_s, high_v]))

            #Morphological opening (remove small objects from the foreground)
            thresholded = cv2.erode(thresholded, np.ones((2,2), np.uint8), iterations=1)
            thresholded = cv2.dilate(thresholded, np.ones((2,2), np.uint8), iterations=1)

            #Morphological closing (fill small holes in the foreground)
            thresholded = cv2.dilate(thresholded, np.ones((2,2), np.uint8), iterations=1)
            thresholded = cv2.erode(thresholded, np.ones((2,2), np.uint8), iterations=1)

            ret,thresh = cv2.threshold(thresholded,157,255,0)

            contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        
            cv2.drawContours(cv_image, contours, -1, (0,255,0), 3)

            numobj = len(contours) # number of objects found in current frame
            #print 'Number of objects found in the current frame: ' , numobj

            if numobj > 0:
                moms = cv2.moments(contours[0])
                if moms['m00']>500:
                    cx = int(moms['m10']/moms['m00'])
                    cy = int(moms['m01']/moms['m00'])

                    #Position in the baxter's stationary base frame
                    xb = (cy - (height/2))*.0023*.433 + .712 + .02
                    yb = (cx - (width/2))*.0023*.433 + .316  - .02
                #print "Found blue ", numobj,  "object(s)" 
                obj_found = True
    else:
        print "Couldn't find any green or blue objects."
     
    #baxter_image = cv_bridge.CvBridge().cv2_to_imgmsg(cv_image, encoding="bgr8")
    #pub.publish(baxter_image)
    # Sleep to allow for image to be published.
    #rospy.sleep(1)

    #Printing to screen the images
    cv2.imshow("Original", cv_image)
    cv2.imshow("Thresholded", thresholded)
    cv2.waitKey(3)

'''
Creates and returns a response with object location info for the object_location_service.
'''
def get_obj_location(request):
    global xb, yb, obj_color, correct_location, obj_found 
    zb = 0
    while xb == 0 and yb == 0:
        rospy.sleep(1)
    return ObjLocationResponse(xb, yb, zb, obj_found, obj_color)

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
    rospy.Subscriber("/cameras/left_hand_camera/image", Image, callback)

    #Declare object location service called object_location_srv with ObjLocation service type.
    #All requests are passed to get_obj_location function
    obj_location_srv = rospy.Service("object_location_service", ObjLocation, get_obj_location)

    


    # print "Left - Ready to find object."

    #Keeps code from exiting until service is shutdown
    rospy.spin()


if __name__ == '__main__':
     main()
