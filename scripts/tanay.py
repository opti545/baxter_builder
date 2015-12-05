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

block1_flag = False
block2_flag = False
block3_flag = False
sleep_flag = True
xpos_up = 0
ypos_up = 0


def InitializeMoveItCommander():
    moveit_commander.roscpp_initialize(sys.argv)
    # Instantiate a RobotCommander object.  This object is an interface to
    # the robot as a whole.
    robot = moveit_commander.RobotCommander()
    rospy.sleep(1)

    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    global scene
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1)

    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the left
    ## arm.  This interface can be used to plan and execute motions on the left
    ## arm.
    global group
    group = MoveGroupCommander("left_arm")
    
    global left_gripper
    left_gripper = baxter_interface.Gripper('left')
    left_gripper.calibrate()

    rospy.sleep(2)

def move_block():
    pose_target = geometry_msgs.msg.Pose()

    left_gripper.open()
    rospy.sleep(2)

    pose_target.orientation.x = 1
    pose_target.position.x = 0.8
    pose_target.position.y = 0.6
    pose_target.position.z = 0.2
    group.set_pose_target(pose_target)
    plan5 = group.plan()
    rospy.sleep(5)
    group.go(wait=True)
    rospy.sleep(2)

    pose_target.orientation.x = 1 
    pose_target.position.x = xpos_up
    pose_target.position.y = ypos_up
    # pose_target.position.x = 0.4
    # pose_target.position.y = 0.1
    pose_target.position.z = 0.15
    group.set_pose_target(pose_target)
    plan1 = group.plan()
    rospy.sleep(5)
    group.go(wait=True)
    rospy.sleep(2)
   


    # rospy.spin()

    # left_gripper.close()
    # rospy.sleep(1)



def read_pos(Point):
    # print "in read_pos"
    global xpos_up, ypos_up
    xpos_up = Point.x
    ypos_up = Point.y
    # print 'xpos: ', xpos_up
    # print 'ypos: ', ypos_up
    # rospy.sleep(3)



def main():
    rospy.init_node('move_block')

    print "===========Initializing MoveIt Commander"
    InitializeMoveItCommander()

    # while not rospy.is_shutdown():
    rospy.Subscriber("/opencv/center_of_object", Point, read_pos)
    rospy.sleep(0.05)

    while not rospy.is_shutdown():
        move_block()

        # rospy.sleep(2)


if __name__ == '__main__':
    main()

#------------------------------------------------------------------------------------

# import sys
# import copy
# import rospy
# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg 
# import baxter_interface
# from baxter_interface import Gripper

# from std_msgs.msg import (Header, String)
# from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
# from baxter_core_msgs.msg import DigitalIOState

# from moveit_commander import MoveGroupCommander

# from std_msgs.msg import Int32
# from geometry_msgs.msg import Pose, PoseStamped

# from baxter_core_msgs.srv import ( SolvePositionIK,
#                                    SolvePositionIKRequest )

# sleep_flag = True
# xpos = 0
# ypos = 0
# zpos = 0

# def init():
#     #Wake up Baxter
#     baxter_interface.RobotEnable().enable()
#     rospy.sleep(0.25)
#     print "Baxter is enabled"
    
#     global ik_service
#     ik_service = rospy.ServiceProxy(
#             "ExternalTools/left/PositionKinematicsNode/IKService",
#             SolvePositionIK)

#     global ik_service_right
#     ik_service_right = rospy.ServiceProxy(
#             "ExternalTools/right/PositionKinematicsNode/IKService",
#             SolvePositionIK)

#     global stopflag
#     stopflag = False
#     #Taken from the MoveIt Tutorials
#     moveit_commander.roscpp_initialize(sys.argv)
#     robot = moveit_commander.RobotCommander()

#     global scene
#     scene = moveit_commander.PlanningSceneInterface()

#     #Activate Left Arm to be used with MoveIt
#     global group
#     group = MoveGroupCommander("left_arm")
    
    
#     global right_group
#     right_group = MoveGroupCommander("right_arm")
#     pose_right = Pose()
#     pose_right.position = Point(0.587, -0.579, 0.480)
#     pose_right.orientation = Quaternion(0.029, 0.998, -0.046, 0.015)
#     request_pose_right(pose_right)
    

#     global left_gripper
#     left_gripper = baxter_interface.Gripper('left')
#     left_gripper.calibrate()

#     move_to_vision()


# def request_pose_right(pose):
#     # Set stamped pose
#     pose_stamped = PoseStamped()
#     pose_stamped.pose = pose
#     pose_stamped.header.frame_id = "base"
#     pose_stamped.header.stamp = rospy.Time.now()

#     # Create IK request 
#     ik_request = SolvePositionIKRequest()
#     ik_request.pose_stamp.append(pose_stamped)

#     # Request service
#     try:
#         rospy.wait_for_service("ExternalTools/right/PositionKinematicsNode/IKService", 5.0)
#         ik_response = ik_service_right(ik_request)
#     except (rospy.ServiceException, rospy.ROSException), error_message:
#         rospy.logerr("Service request failed: %r" %(error_message))
#         sys.exit("ERROR - move_to_observe - Failed to append pose")
#     if ik_response.isValid[0]:
#         limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
#         right_group.set_joint_value_target(limb_joints)
#         plan2= right_group.plan()
#         rospy.sleep(5)
#         right_group.go(wait=True)


# def request_pose(pose):
#     # Set stamped pose
#     pose_stamped = PoseStamped()
#     pose_stamped.pose = pose
#     pose_stamped.header.frame_id = "base"
#     pose_stamped.header.stamp = rospy.Time.now()

#     # Create IK request 
#     ik_request = SolvePositionIKRequest()
#     ik_request.pose_stamp.append(pose_stamped)

#     # Request service
#     try:
#         rospy.wait_for_service("ExternalTools/left/PositionKinematicsNode/IKService", 5.0)
#         ik_response = ik_service(ik_request)
#     except (rospy.ServiceException, rospy.ROSException), error_message:
#         rospy.logerr("Service request failed: %r" %(error_message))
#         sys.exit("ERROR - move_to_observe - Failed to append pose")
#     if ik_response.isValid[0]:
#         limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
#         group.set_joint_value_target(limb_joints)
#         plan2= group.plan()
#         rospy.sleep(5)
#         group.go(wait=True)
#         rospy.sleep(3)

# def move_to_vision():
#     # Set pose
#     pose = Pose()
#     pose.orientation = Quaternion(0.00, 1.0, 0.00, 0.00)
#     pose.position = Point(0.7, 0.316, 0.1)

#     # Request service
#     request_pose(pose)


# def move_to_box(x, y):
#     pose = Pose()
#     pose.orientation = Quaternion(0.00, 1.0, 0.00, 0.00)
#     pose.position = Point(xpos, ypos, 0.1)
#     request_pose(pose)


# def pick_and_place(xposl, yposl, zposl):
#     pose = Pose()
#     pose.position = Point(xposl, yposl, 0.250)
#     pose.orientation = Quaternion(0.00, 1.00, 0.00, 0.00)

#     request_pose(pose)
#     left_gripper.close()
#     move_to_box()
#     left_gripper.open()
#     move_to_vision()
#     sleep_flag = True


# def read_pos(msg):

#     global xpos, ypos, zpos
#     # global sleep_flag

#     # if sleep_flag == True:  
#     xpos = msg.x
#     ypos = msg.y
#     zpos = 0.1
#     # print "Using these values"
#     # print msg
#         # sleep_flag = False

# def main():
#     rospy.init_node('baxter_mover_node')

#     print "Initializing all MoveIt related functions"
#     init()

#     print "Subscribing to center_of_object topic to receive points"
#     rospy.Subscriber("/opencv/center_of_object", Point, read_pos)
#     rospy.sleep(1)
   
#     # count = 1
    
#     # while not rospy.is_shutdown():
#     #     global sleep_flag
#     #     while sleep_flag:
#     #         pass

#     #     pick_and_place(xpos, ypos, zpos)

#     #     sleep_flag = True

#     while not rospy.is_shutdown():
#         # move_to_vision()
#         move_to_box(0.7, 0.1)



# if __name__ == '__main__':
#     main()