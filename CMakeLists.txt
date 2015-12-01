cmake_minimum_required(VERSION 2.8.3)
project(baxter_builder)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    baxter_core_msgs
    baxter_interface
    baxter_maintenance_msgs
    control_msgs
    cv_bridge
    geometry_msgs
    roscpp
    rospy
    sensor_msgs
    trajectory_msgs
    moveit_core
    moveit_ros_planning_interface
)

catkin_package()

include_directories(
    ${catkin_INCLUD_LIBS}
)
