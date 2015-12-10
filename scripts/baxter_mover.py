#!/usr/bin/env python


import sys
import copy
import rospy
import cv2
import cv_bridge
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg 
import baxter_interface
from baxter_interface import Gripper
import numpy as np

from std_msgs.msg import (Header, String)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from baxter_core_msgs.msg import EndEffectorState

from sensor_msgs.msg import Image
from moveit_commander import MoveGroupCommander

from std_msgs.msg import Int32
from geometry_msgs.msg import Pose, PoseStamped

from baxter_core_msgs.srv import ( SolvePositionIK,
                                   SolvePositionIKRequest )

from baxter_builder.srv import *

sleep_flag = True
xpos = 0
ypos = 0
zpos = 0
grip_force = 0

def init():
    #Wake up Baxter
    baxter_interface.RobotEnable().enable()
    rospy.sleep(0.25)
    print "Baxter is enabled"
    
    print "Intitializing clients for services"
    global ik_service_left
    ik_service_left = rospy.ServiceProxy(
            "ExternalTools/left/PositionKinematicsNode/IKService",
            SolvePositionIK)

    global ik_service_right
    ik_service_right = rospy.ServiceProxy(
            "ExternalTools/right/PositionKinematicsNode/IKService",
            SolvePositionIK)

    global obj_loc_service 
    obj_loc_service = rospy.ServiceProxy(
        "object_location_service", ObjLocation)

    global stopflag
    stopflag = False
    #Taken from the MoveIt Tutorials
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()

    global scene
    scene = moveit_commander.PlanningSceneInterface()

    #Activate Left Arm to be used with MoveIt
    global left_group
    left_group = MoveGroupCommander("left_arm")
    left_group.set_goal_position_tolerance(0.01)
    left_group.set_goal_orientation_tolerance(0.01)
    
    
    global right_group
    right_group = MoveGroupCommander("right_arm")
    pose_right = Pose()
    pose_right.position = Point(0.793, -0.586, 0.329)
    pose_right.orientation = Quaternion(1.0, 0.0, 0.0, 0.0)
    request_pose(pose_right, "right", right_group)
    

    global left_gripper
    left_gripper = baxter_interface.Gripper('left')
    left_gripper.calibrate()
    print left_gripper.parameters()

    global end_effector_subs
    end_effector_subs = rospy.Subscriber("/robot/end_effector/left_gripper/state", EndEffectorState, end_effector_callback)
    rospy.sleep(1)

    global pubpic
    pubpic = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    rospy.sleep(1)

def request_pose(pose, arm, groupl):
    # Set stamped pose
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.frame_id = "base"
    pose_stamped.header.stamp = rospy.Time.now()

    # Create IK request 
    ik_request = SolvePositionIKRequest()
    ik_request.pose_stamp.append(pose_stamped)

    arm_name_service = "ExternalTools/" + arm + "/PositionKinematicsNode/IKService"
    # Request service
    try:
        rospy.wait_for_service(arm_name_service, 5.0)
        if arm == "left":
            ik_response = ik_service_left(ik_request)
        else:
            ik_response = ik_service_right(ik_request)
    except (rospy.ServiceException, rospy.ROSException), error_message:
        rospy.logerr("Service request failed: %r" %(error_message))
        sys.exit("ERROR - move_to_observe - Failed to append pose")
    if ik_response.isValid[0]:
        limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
        groupl.set_start_state_to_current_state()
        groupl.set_joint_value_target(limb_joints)
        plan2= groupl.plan(limb_joints)
        rospy.sleep(1.5)
        groupl.execute(plan2)
        rospy.sleep(0.25)
        return True
    else:
        return False

def move_to_vision():
    # Set pose
    pose = Pose()
    pose.orientation = Quaternion(1.00, 0.0, 0.00, 0.00)
    pose.position = Point(0.712, 0.316, 0.250)

    # Request service
    request_pose(pose,"left", left_group)

def end_effector_callback(msg):
    global grip_force
    grip_force = msg.force

def move_to_box(objcolorl):
    if objcolorl == 0:
        #move to green box
        pose = Pose()
        pose.orientation = Quaternion(1.00, 0.0, 0.00, 0.00)
        pose.position = Point(0.570, -0.176, 0.283)
        path = '/home/josmiranda/bt_ws/src/baxter_builder/images/green_success.png' 
        img = cv2.imread(path)

        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        pubpic.publish(msg)
    else:
        #move to blue box
        pose = Pose()
        pose.orientation = Quaternion(1.00, 0.0, 0.00, 0.00)
        pose.position = Point(0.708, -0.153, 0.258)
        path = '/home/josmiranda/bt_ws/src/baxter_builder/images/red_success.png' 
        img = cv2.imread(path)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        pubpic.publish(msg)
    request_pose(pose, "left", left_group)


def move_to_object(xposl, yposl, zposl, objcolorl):
    global grip_force
    pose = Pose()
    pose.position = Point(xposl-0.01, yposl, 0.00)
    pose.orientation = Quaternion(1.00, 0.0, 0.00, 0.00)


    request_pose(pose, "left", left_group)
    rospy.sleep(0.5)

    # Get left hand range state
    dist = baxter_interface.analog_io.AnalogIO('left_hand_range').state()
    rospy.sleep(1)
    if dist > 65000:
        print "Out of Range"
        truez = -0.13
    else:
        print "DISTANCE %f" % dist
        truez = dist/1000
        truez = truez - 0.06
        truez = - (truez)
        print truez


    poset = Pose()
    poset.position = Point(xposl, yposl, truez)
    poset.orientation = Quaternion(1.00, 0.0, 0.00, 0.00)
    request_pose(poset, "left", left_group)

    left_gripper.close()
    rospy.sleep(0.5)
    if grip_force == 0:
        left_gripper.open()
        move_to_vision()
    else:
        pose = Pose()
        pose.position = Point(xposl-0.01, yposl+0.01, 0.150)
        pose.orientation = Quaternion(1.00, 0.0, 0.00, 0.00)
        request_pose(pose, "left", left_group)
        if grip_force == 0:
            left_gripper.open()
            move_to_vision()
            return
        move_to_box(objcolorl)
        left_gripper.open()
        move_to_vision()


def main():
    rospy.init_node('baxter_mover_node')
    print "Initializing all MoveIt related functions and services"
    init()
    print "Move to Vision Pose"
    move_to_vision()
    rospy.sleep(15)
    while not rospy.is_shutdown():
        try:
            rospy.wait_for_service('object_location_service')
            response = obj_loc_service.call(ObjLocationRequest())
            print response
            if response.objfound == True:
                move_to_object(response.xb, response.yb, response.zb, response.objcolor)
            else:
                #Move to random pose
                print "Moving to random pose"
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

if __name__ == '__main__':
    main()
