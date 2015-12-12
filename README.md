Welcome to the Baxter Builder Project!
===================

Northwestern University
ME495: Embedded Systems in Robotics (Fall 2015)
Instructor:  Jarvis Schultz 

Team members: Jose Miranda, Sofya Akhmametyeva, Yves Nazon and Tanay Choudhary

### Table of contents
- [Overview](#Overview)
- [Required tools and set up](#Required tools and set up)
- [left_camera_node](#left_camera_node)
- [move_arm_node](#move_arm_node)
- [Gazebo world](#Gazebo world)
- [Launch file](#Launch file)

<a name="Overview"></a> 
Overview
-------------

The goal of the project was to have Baxter detect objects via left hand camera and sort them into two boxes based on green and red colors. MoveIt! package and IK Service were leveraged for the movement and path planning. Open CV library was used for the image processing and object detection.

Please click on the following image to watch the demo!
[![Screenshot](https://raw.githubusercontent.com/opti545/baxter_builder/perception_branch/pictures/baxter_builder.PNG)](https://www.youtube.com/watch?v=7gfVH0BbgBk)

----------

<a name="Required tools and set up"></a> 
Required tools and set up
-------------
- Baxter Robot 
- [ROS Indigo](http://wiki.ros.org/ROS/Installation) for Ubuntu 14.04 
- [Baxter Setup](http://sdk.rethinkrobotics.com/wiki/Baxter_Setup)
- ROS Environment variables must be set properly in order to be able to connect and interface with baxter. It can be useful to add the following lines of code into your .bashrc:

```
#for baxter simulation
alias bxl="unset ROS_IP; unset ROS_HOSTNAME; unset ROS_MASTER_URI; export ROS_IP=127.0.0.1; export ROS_MASTER_URI=http://localhost:11311"
#for actual baxter
alias bxb="unset ROS_IP; unset ROS_HOSTNAME; unset ROS_MASTER_URI; export ROS_IP=10.42.0.1; export ROS_MASTER_URI=http://baxter.local:11311"
```
------------

<a name="left_camera_node"></a> 
left_camera_node
-------------

The *left_vision_obj_location.py* script instantiates the left_camera_node. This node subscribes to the */cameras/left_hand_camera/image* topic and performs image processing, finds the location of a desired object and provides that information via a custom *ObjLocation service* . In more detail about the various packages and services that were used for the image processing, please see below:

***Open CV***

  We used **cv2** package for image processing, as well as **CvBridge** in order to be able to interface with ROS and Baxter. The left hand camera image was captured, converted to cv2 image and then converted to HSV format as follows:

  ```
  bridge = CvBridge()
  cv_image = bridge.imgmsg_to_cv2(message, "bgr8")
  #Converting image to HSV format
  hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
  ```

  Later the cv_image was thresholded with green and red color HSV ranges in order to pick out the desired objects. Here is a quick example: 

  ```
  thresholded = cv2.inRange(hsv, np.array([low_h, low_s, low_v]), np.array([high_h, high_s, high_v]))
  #Morphological opening (remove small objects from the foreground)
  thresholded = cv2.erode(thresholded, np.ones((2,2), np.uint8), iterations=1)
  thresholded = cv2.dilate(thresholded, np.ones((2,2), np.uint8), iterations=1)
  #Morphological closing (fill small holes in the foreground)
  thresholded = cv2.dilate(thresholded, np.ones((2,2), np.uint8), iterations=1)
  thresholded = cv2.erode(thresholded, np.ones((2,2), np.uint8), iterations=1)
  #cv2.threshold(source image in greyscale, threshold value which is used to classify the pixel values, the maxVal which represents the value to be given if pixel value is more than (sometimes less than) the threshold value )
  #output: retval and thresholded image
  ret,thresh = cv2.threshold(thresholded,157,255,0)
``` 
  Contours of the objects were found and used for the calculation of the centroid of each object. They were also drawn on top of the original image for a nice visual:

  ```
  contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
  cv2.drawContours(cv_image, contours, -1, (0,255,0), 3)
  # number of objects found in current frame
  numobj = len(contours) 
  if numobj > 0:
      moms = cv2.moments(contours[0])
      if moms['m00']>500:
           cx = int(moms['m10']/moms['m00'])
           cy = int(moms['m01']/moms['m00'])
  ```


---
***Object position calculation***

  Once the contours of the objects are found, the centroids of those objects are calculated in the camera frame. Then the coordinates are changed from left hand camera's moving frame to Baxter's stationary base frame, as follows:

  ```
  #Position of the object in the baxter's moving camera frame 
  cx = centres[0][0]
  cy = centres[0][1]
  pix_size = .0023 #Camera calibration (meter/pixels). Pixel size at 1 meter.
  h = .433 #Height from table/obj to camera
  x0b = .712 # x position of initial position in baxter's base frame 
  y0b = .316 # y position of initial position in baxter's base frame 
  x_camera_offset = .02 #x camera offset from center of the gripper  
  y_camera_offset = -.02 #y camera offset from center of the gripper 
  # height, width, depth = cv_image.shape #camera frame dimensions 
  #Position of the object in the baxter's stationary base frame
  xb = (cy - (height/2))*pix_size*h + x0b + x_camera_offset 
  yb = (cx - (width/2))*pix_size*h + y0b  + y_camera_offset
  ```

---
***Object Location Service Provider***

  The *left_camera_node* provides ***object_location_service***. When *move_arm_node* sends an (empty) request, *left_camera_node* tries to find the desired object within a current camera frame via image processing and send back a response with the following structure:

 - float32 xb 
   - X coordinate of the centroid of the desired object in the Baxter's base frame.
 - float32 yb 
   - Y coordinate of the centroid of the desired object in the Baxter's base frame.
 - float32 zb
   - Z coordinate of the centroid of the desired object in the Baxter's base frame. (Currently not using it)
 - bool objfound 
   - True if green or blue objects were found and False otherwise.
 - int8 objcolor
   - Used for our perception sorting multiplier (currently green or red object). 

----
***Topics Subscribed*** 

  The **left_camera_node** subscribes to the following topic with the Baxter's left hand camera feed:

  ``` 
  /cameras/left_hand_camera/image
  ```

  This camera image is later processed using Open CV libraries, as described above, in order to extract a centroid location of a desired object.

-----


<a name="move_arm_node"></a>
move_arm_node 
-------------

All related Baxter movements and inverse kinematics calculations were done in the **move_arm_node**. There are various packages and services that we used for the movement. They are as follow:

***MoveIt!***

  We were able to use the MoveIt! package to work with Baxter. Essentially, this package is powerful enough for motion planning and manipulation and it makes an easy interface to work with Baxter. There are some various Python Interfaces with MoveIt! that we took in for our advantage. The most important interface that we took from MoveIt! is the **moveit_commander**. This package allowed us to control Baxter's arm by simply calling for example the following:

```left_group = MoveGroupCommander("left_arm")```

  This allowed us to use ***left_group*** for saying things like plan a joint state plan and execute it using simple things as follow:  
```
      plan2= left_group.plan(limb_joints)
          rospy.sleep(1.5)
          left_group.execute(plan2)
          rospy.sleep(0.25)
  ``` 

  In addition, we were able to use one of the recent released packages for Inverse Kinematics called [trac_ik](http://www.ros.org/news/2015/11/introducing-a-better-inverse-kinematics-package.html).  The reason for using this package was to allow better planning calculations with MoveIt! since we used it as the IK solver for the **moveit_commander** interface instead of its default IK solver plugin. More information in installing the package can be found in [here](http://www.ros.org/news/2015/11/introducing-a-better-inverse-kinematics-package.html). 

--- 

***IK Service***

  As part of our inverse kinematics calculations, we used a service provided by ROS that allowed us to compute the joint values for a specific target pose. This was used in conjunction with MoveIt! to allow us to get better accuracy for the arm to reach the specified pose.  
  A sample code taken from our ***move_arm_node*** can be seen so it can be easier to see how we use these services:

```
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
```
---
***Object Location Service Client***

  Within our framework, we used the Service Server/Client infrastructure to get target poses of our objects to pick up. The ***move_arm_node*** was the client requesting a location of objects through an empty message request. The response that the node gets contains information about whether a green/red object was found in the current field of vision, the color and the (x,y,z) location of the object.


---
***Topics Subscribed*** 

  Within our ***move_arm_node***, there are several topics we subscibred to for a variety of features that we offered. They are as follow:

  ``` 
  /robot/end_effector/left_gripper/state
  ```

  The above topic is used to detect the force applied to the left gripper of Baxter's arm so that we can use it to determine if it grabbed the block or not. If not, then we can return to the vision pose and not do the whole motion of dropping empty things and going back to the vision pose (basically to minimize  trajectory execution if we did not grab something).

  ```
  /robot/xdisplay
  ```

  This topic was used to display images to the head monitor of Baxter. There are some hard coded links for the pictures that may run into errors when trying to run our nodes. Changing the path of the image might be better solution, or just commenting the portion where it publishes the image.

---


<a name="Gazebo world"></a> 
Gazebo world
-------

Due to the limited number of Baxters available for testing we turned to computer simulation,
specifically Gazebo, for early validation of our nodes. We started with the baxter.world file that
included a complete model of the Baxter robot. From there we created a custom Gazebo world
using Gazebo's GUI to add competition relevant objects such as colored blocks, balls, and tables to
our world. Next we edited the newly constructed SDF file to properly model the vision and
collision properties of these objects. Finally we manually configured the world objects in various
manners to see how Baxter dealt with different testing configurations. Gazebo allowed us to
rapidly iterate our code by giving us the ability to immediately see how changes to our node would
affect Baxter's performance. While simulation is never a perfect representation of the real world, it
was a great starting place for us.



-------


<a name="Launch file"></a> 
Launch file
---
In order to run the project, type the following:

```roslaunch baxter_builder setup.launch```

The setup.launch file will start up both the  move_arm_node and the left_camera_node, as well as the baxter's interface trajectory_node and the necessary setup for the MoveIt configuration. Here is the setup.launch code:

```

  <launch>
    <arg name="config" default="true"/>

    <!--Node for trajectories, used with MoveIt -->
    <node pkg="baxter_interface" type="joint_trajectory_action_server.py" name="trajectory_node" output="log" >
    </node>   

    <!-- Taken from the demo_baxter.launch in the baxter_moveit_config-->

    <include file="$(find baxter_moveit_config)/launch/planning_context.launch">
      <arg name="load_robot_description" value="true"/>
    </include>

    <arg name="kinect" default="false" />
    <arg name="xtion" default="false" />
    <arg name="camera_link_pose" default="0.15 0.075 0.5 0.0 0.7854 0.0"/>
    <include file="$(find baxter_moveit_config)/launch/move_group.launch">
      <arg name="kinect" value="$(arg kinect)" />
      <arg name="xtion" value="$(arg xtion)" />
      <arg name="camera_link_pose" default="$(arg camera_link_pose)"/>
      <arg name="allow_trajectory_execution" value="true"/>
    </include>

    <!--Start the move_arm node from our scripts -->
    <node pkg="baxter_builder" type="baxter_mover.py" name="move_arm_node" output ="screen">
    </node>

    <!--Node that uses camera to find block -->
    <node pkg="baxter_builder" type="left_vision_obj_location.py" name="camera_node" output="screen">
    </node>
  </launch> 
```
---
