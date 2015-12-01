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

global block1_flag, block2_flag, block3_flag
global sleep_flag
global xpos_up,ypos_up
global group
global scene
global left_gripper
global pick_flag
block1_flag = False
block2_flag = False
block3_flag = False
sleep_flag = True
xpos_up = 0
ypos_up = 0
pick_flag = False


def init():
    moveit_commander.roscpp_initialize(sys.argv)
    # Instantiate a RobotCommander object.  This object is an interface to
    # the robot as a whole.
    robot = moveit_commander.RobotCommander()
    # rospy.sleep(1)

    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the left
    ## arm.  This interface can be used to plan and execute motions on the left
    ## arm.
    
    group = MoveGroupCommander("left_arm")
    
    left_gripper = baxter_interface.Gripper('left')
    left_gripper.calibrate()

    rospy.sleep(2)

def move_block(xpos_up, ypos_up, zpos_up, zpos_down):
    print "============ Generating Block Pick Up Plan"
    pose_target = geometry_msgs.msg.Pose()

    pose_target.orientation.x = 1 
    pose_target.position.x = xpos_up
    pose_target.position.y = ypos_up
    pose_target.position.z = zpos_up
    group.set_pose_target(pose_target)
    plan1 = group.plan()
    group.go(wait=True)
    rospy.sleep(2)

    #Close Baxter's left gripper
    left_gripper.close()
    rospy.sleep(2)

    print "============ Generating waypoint plan"
    pose_target.orientation.x = 1
    pose_target.position.x = 0.7
    pose_target.position.y = 0.4
    pose_target.position.z = 0.3
    group.set_pose_target(pose_target)
    plan2 = group.plan()
    group.go(wait=True)
    rospy.sleep(2)


    print "============ Generating block placement plan"
    pose_target.orientation.x = 1
    pose_target.position.x = 0.83
    pose_target.position.y = 0.1
    pose_target.position.z = zpos_down 
    group.set_pose_target(pose_target)
    plan3 = group.plan()
    group.go(wait=True)
    rospy.sleep(2)


    #Close Baxter's left gripper
    left_gripper.open()
    rospy.sleep(2)


    #Waypoint to vision place
    print "============ Generating return plan"
    pose_target.orientation.x = 1
    pose_target.position.x = 0.85
    pose_target.position.y = 0.1
    pose_target.position.z = 0.1
    group.set_pose_target(pose_target)
    plan4 = group.plan()
    group.go(wait=True)
    rospy.sleep(2)

    #moving back to vision place
    print "============ Back to Vision Position"
    pose_target.orientation.x = 1
    pose_target.position.x = 0.7
    pose_target.position.y = 0.5
    pose_target.position.z = 0.3
    group.set_pose_target(pose_target)
    plan5 = group.plan()
    group.go(wait=True)
    rospy.sleep(2)


def read_pos(Point):
    # print "in read_pos"
    global pick_flag
    xpos_up = Point.x
    ypos_up = Point.y
    pick_flag = True



def block_counter(count):
    print "In block_counter"
    
    if count == 1:
        print "Setting block1_flag"
        block1_flag = True
    elif count == 2:
        print "Setting block2_flag"
        block2_flag = True
    elif count == 3:
        print "Setting block3_flag"
        block3_flag = True


def checkButton(msg):
    global sleep_flag
    sleep_flag = True
    if msg.state == True:
        sleep_flag = False


def main():
    rospy.init_node('move_arm_node')

    print 'Init function'
    init()

    #finds postion of green objects
    rospy.Subscriber("/opencv/center_of_object", Point, read_pos)
    rospy.sleep(0.05)
    global pick_flag
    pick_flag = False
    count = 1
    while not rospy.is_shutdown():
        block_counter(count)
        rospy.sleep(1)

        #print xpos_up, ypos_up, zpos_up, zpos_down
        if pick_flag == True:
            
            move_block(xpos_up, ypos_up, zpos_up, zpos_down)
            pick_flag = False

        count = count +1

        global sleep_flag
        sleep_flag = True

        if count > 3:
            #time to reset blocks
            print "==========Sleeping"

            while sleep_flag:
                pass
            
            count = 1


if __name__ == '__main__':
    main()
