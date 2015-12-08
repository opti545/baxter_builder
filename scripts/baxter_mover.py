#!/usr/bin/env python


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg 
import baxter_interface
from baxter_interface import Gripper

from std_msgs.msg import (Header, String)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from baxter_core_msgs.msg import DigitalIOState

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

global group
global right_group
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
    
    
    global right_group
    right_group = MoveGroupCommander("right_arm")
    pose_right = Pose()
    pose_right.position = Point(0.587, -0.579, 0.480)
    pose_right.orientation = Quaternion(0.029, 0.998, -0.046, 0.015)
    request_pose(pose_right, "right", right_group)
    

    global left_gripper
    left_gripper = baxter_interface.Gripper('left')
    left_gripper.calibrate()


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
        #groupl.clear_pose_targets()
        groupl.set_start_state_to_current_state()
        groupl.set_joint_value_target(limb_joints)
        plan2= groupl.plan(limb_joints)
        rospy.sleep(3)
        groupl.go(wait=True)


def move_to_vision():
    # Set pose
    pose = Pose()
    pose.orientation = Quaternion(1.00, 0.0, 0.00, 0.00)
    pose.position = Point(0.712, 0.316, 0.250)

    # Request service
    request_pose(pose,"left", left_group)


def move_to_box():
    pose = Pose()
    pose.orientation = Quaternion(1.00, 0.0, 0.00, 0.00)
    pose.position = Point(0.737, -0.114, 0.283)
    request_pose(pose, "left", left_group)


def move_to_object(xposl, yposl, zposl, zready = False):
    pose = Pose()
    pose.position = Point(xposl, yposl, 0.150)
    pose.orientation = Quaternion(1.00, 0.0, 0.00, 0.00)

    request_pose(pose, "left", left_group)
    rospy.sleep(2)

    poset = Pose()
    poset.position = Point(xposl, yposl, -0.13)
    poset.orientation = Quaternion(1.00, 0.0, 0.00, 0.00)
    request_pose(poset, "left", left_group)
    left_gripper.close()
    move_to_box()
    left_gripper.open()


def main():
    rospy.init_node('baxter_mover_node')
    print "Initializing all MoveIt related functions and services"
    init()
    print "Move to Vision Pose"
    move_to_vision()
    global object_location_calc
    object_location_calc = True
    while not rospy.is_shutdown():
        try:
            rospy.wait_for_service('object_location_service')
            response = obj_loc_service.call(ObjLocationRequest(object_location_calc))
            print response
            if response.objfound == True:
                move_to_object(response.xb, response.yb, response.zb, response.zflag)
                move_to_vision()
            else:
                #Move to random pose
                print "Moving to random pose"
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

if __name__ == '__main__':
    main()
