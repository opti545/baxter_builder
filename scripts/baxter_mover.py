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

sleep_flag = True
xpos = 0
ypos = 0
zpos = 0

def init():
    #Wake up Baxter
    baxter_interface.RobotEnable().enable()
    rospy.sleep(0.25)
    print "Baxter is enabled"
    
    global ik_service
    ik_service = rospy.ServiceProxy(
            "ExternalTools/left/PositionKinematicsNode/IKService",
            SolvePositionIK)

    global ik_service_right
    ik_service_right = rospy.ServiceProxy(
            "ExternalTools/right/PositionKinematicsNode/IKService",
            SolvePositionIK)
    
    global stopflag
    stopflag = False
    #Taken from the MoveIt Tutorials
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()

    global scene
    scene = moveit_commander.PlanningSceneInterface()

    #Activate Left Arm to be used with MoveIt
    global group
    group = MoveGroupCommander("left_arm")
    
    
    global right_group
    right_group = MoveGroupCommander("right_arm")
    pose_right = Pose()
    pose_right.position = Point(0.587, -0.579, 0.480)
    pose_right.orientation = Quaternion(0.029, 0.998, -0.046, 0.015)
    request_pose_right(pose_right)
    

    global left_gripper
    left_gripper = baxter_interface.Gripper('left')
    left_gripper.calibrate()

    move_to_vision()


def request_pose_right(pose):
    # Set stamped pose
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.frame_id = "base"
    pose_stamped.header.stamp = rospy.Time.now()

    # Create IK request 
    ik_request = SolvePositionIKRequest()
    ik_request.pose_stamp.append(pose_stamped)

    # Request service
    try:
        rospy.wait_for_service("ExternalTools/right/PositionKinematicsNode/IKService", 5.0)
        ik_response = ik_service_right(ik_request)
    except (rospy.ServiceException, rospy.ROSException), error_message:
        rospy.logerr("Service request failed: %r" %(error_message))
        sys.exit("ERROR - move_to_observe - Failed to append pose")
    if ik_response.isValid[0]:
        limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
        right_group.set_joint_value_target(limb_joints)
        plan2= right_group.plan()
        rospy.sleep(5)
        right_group.go(wait=True)


def request_pose(pose):
    # Set stamped pose
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.frame_id = "base"
    pose_stamped.header.stamp = rospy.Time.now()

    # Create IK request 
    ik_request = SolvePositionIKRequest()
    ik_request.pose_stamp.append(pose_stamped)

    # Request service
    try:
        rospy.wait_for_service("ExternalTools/left/PositionKinematicsNode/IKService", 5.0)
        ik_response = ik_service(ik_request)
    except (rospy.ServiceException, rospy.ROSException), error_message:
        rospy.logerr("Service request failed: %r" %(error_message))
        sys.exit("ERROR - move_to_observe - Failed to append pose")
    if ik_response.isValid[0]:
        limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
        group.set_joint_value_target(limb_joints)
        plan2= group.plan()
        rospy.sleep(5)
        group.go(wait=True)


def move_to_vision():
    # Set pose
    pose = Pose()
    pose.orientation = Quaternion(0.00, 1.0, 0.00, 0.00)
    pose.position = Point(0.765, 0.035, 0.435)

    # Request service
    request_pose(pose)

def move_to_box():
    pose = Pose()
    pose.orientation = Quaternion(0.00, 1.0, 0.00, 0.00)
    pose.position = Point(0.675, -0.138, 0.412)


def pick_and_place():
    pose = Pose()
    pose.position = Point(xpos, ypos, zpos)
    pose.orientation = Quaternion(0.00, 1.00, 0.00, 0.00)

    request_pose(pose)
    left_gripper.close()

    move_to_vision()
    move_to_box()




def read_pos(msg):
    # print "in read_pos"
    print "Reading from Callback"
    print msg

    global xpos, ypos, zpos
    global sleep_flag

    xpos = msg.x
    ypos = msg.y
    zpos = msg.z

    sleep_flag = False

def main():
    rospy.init_node('baxter_mover_node')

    print "Initializing all MoveIt related functions"
    init()

    print "Subscribing to center_of_object topic to receive points"
    rospy.Subscriber("/opencv/center_of_object", Point, read_pos)
    rospy.sleep(1)
   
    count = 1
    
    while not rospy.is_shutdown():
        global sleep_flag
        while sleep_flag:
            pass

        #move_to_block(xpos, ypos, zpos)

        sleep_flag = True


if __name__ == '__main__':
    main()
