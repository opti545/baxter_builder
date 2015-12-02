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

sleep_flag = True
xpos = 0
ypos = 0
zpos = 0


def init():
    #Wake up Baxter
    baxter_interface.RobotEnable().enable()
    rospy.sleep(0.25)
    print "Baxter is enabled"
    
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
    
    global left_gripper
    left_gripper = baxter_interface.Gripper('left')
    left_gripper.calibrate()

    pose_target = geometry_msgs.msg.Pose()
     #moving back to vision place
    print "Moving to Vision State"
    pose_target.orientation = Quaternion(-0.009, 0.999, 0.002, -0.051)
    pose_target.position = Point(0.627, 0.272, 0.228)
    group.set_pose_target(pose_target)
    plan5 = group.plan()
    rospy.sleep(2)
    group.go(wait=True)
    group.clear_pose_targets()


def move_to_block(xposl, yposl, zposl):
    
    pose_target = geometry_msgs.msg.Pose()

    print "Going to middle pose"
    pose_target.orientation.x = 1
    pose_target.position.x = xposl
    pose_target.position.y = yposl
    pose_target.position.z = zposl
    group.set_pose_target(pose_target)
    plan2 = group.plan()
    rospy.sleep(2)
    group.go(wait=True)
    rospy.sleep(2)

    #Close Baxter's left gripper
    left_gripper.close()
    rospy.sleep(2)


    #Waypoint to vision place
    print "Moving to Vision State"
    pose_target.orientation = Quaternion(-0.009, 0.999, 0.002, -0.051)
    pose_target.position = Point(0.627, 0.272, 0.228)
    group.set_pose_target(pose_target)
    plan4 = group.plan()
    rospy.sleep(2)
    group.go(wait=True)
    rospy.sleep(2)



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

def checkButton(msg):
    global sleep_flag
    sleep_flag = True
    if msg.state == True:
        sleep_flag = False


def main():
    rospy.init_node('baxter_mover_node')

    print "Initializing all MoveIt related functions"
    init()

    print "Subscribing to center_of_object topic to receive points"
    rospy.Subscriber("/opencv/center_of_object", Point, read_pos)
    rospy.sleep(0.5)
   
    count = 1
    
    while not rospy.is_shutdown():
        
        while sleep_flag:
            pass

        move_to_block(xpos, ypos, zpos)
        count = count +1

        global sleep_flag
        sleep_flag = True


if __name__ == '__main__':
    main()
