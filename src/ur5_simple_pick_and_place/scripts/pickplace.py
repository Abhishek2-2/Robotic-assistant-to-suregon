#! /usr/bin/env python
import sys
import rospy
import tf
import math
import os
import moveit_commander
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String



moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_rarm', anonymous=True)
robot = moveit_commander.RobotCommander()

arm_group = moveit_commander.MoveGroupCommander("ur5_arm")
hand_group = moveit_commander.MoveGroupCommander("gripper")



#joint_goal = arm_group.get_current_joint_values()
#joint_goal[0] = -2.25
#joint_goal[1] = -pi/4
#joint_goal[2] = 0
#joint_goal[3] = -pi/2
#joint_goal[4] = 0


# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
#arm_group.go(joint_goal, wait=True)

# Put the arm in the start position
arm_group.set_named_target("home")
plan1 = arm_group.go()

#Open the gripper
#hand_group.set_named_target("close")
#plan2 = hand_group.go()
#rospy.sleep(10)
#Move the arm above the object to be grasped
#Temporarily hardcoded, define pose target by subscribing to the camera module, which in turn subscribes to the speech module

#PICK POSE
#pose_target = geometry_msgs.msg.Pose()
#pose_target.orientation.w = 0.1132710
#pose_target.orientation.x = -0.9935233
#pose_target.orientation.y = -0.00839178
#pose_target.orientation.z = -0.003253623
#pose_target.position.x = 0.004868729
#pose_target.position.y = 0.15509091
#pose_target.position.z = 0.08051772
#arm_group.set_pose_target(pose_target)
#arm_group.set_goal_tolerance(0.001)
#arm_group.set_goal_orientation_tolerance(0.001)
#arm_group.set_goal_position_tolerance(0.001)
#plan1 = arm_group.go()

#RANDOM POSE
#pose_target = geometry_msgs.msg.Pose()
#pose_target.orientation.w = -0.10465534
##pose_target.orientation.x = -0.36673992
#pose_target.orientation.y = 0.7767989
#pose_target.orientation.z = -0.5011311
#pose_target.position.x = -0.02523177
#pose_target.position.y = 0.0107332
#pose_target.position.z = 0.2711836
#arm_group.set_pose_target(pose_target)
#arm_group.set_goal_tolerance(0.001)
#arm_group.set_goal_orientation_tolerance(0.001)
#arm_group.set_goal_position_tolerance(0.001)
#plan1 = arm_group.go()




#rospy.init_node('ur5_arm', anonymous=True)
listener = tf.TransformListener()

listener.waitForTransform("/base_link", "/logical_camera_2_unit_box_blue_clone_frame", rospy.Time(), rospy.Duration(10.0))
(trans, rot) = listener.lookupTransform('base_link', 'logical_camera_2_unit_box_blue_clone_frame', rospy.Time())
current_pose = arm_group.get_current_pose()
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation = current_pose.pose.orientation
pose_target.position.x = trans[0]
pose_target.position.y = trans[1]
pose_target.position.z = 0.077
arm_group.set_pose_target(pose_target)
    #arm_group.set_goal_tolerance(0.001)
    #arm_group.set_goal_orientation_tolerance(0.001)
    #arm_group.set_goal_position_tolerance(0.001)
plan1 = arm_group.go()
    
pose_target.position.z = 0.002
arm_group.set_pose_target(pose_target)
plan1 = arm_group.go()
    

hand_group.set_named_target("closed")
plan2 = hand_group.go()
    

pose_target.position.z = 0.067
arm_group.set_pose_target(pose_target)
plan1 = arm_group.go()
    

listener.waitForTransform("/base_link", "/logical_camera_2_bowl_frame", rospy.Time(), rospy.Duration(10.0))
(trans2, rot2) = listener.lookupTransform('base_link', 'logical_camera_2_bowl_frame', rospy.Time())

current_pose = arm_group.get_current_pose()
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation = current_pose.pose.orientation
pose_target.position.x = trans2[0]
pose_target.position.y = trans2[1]
pose_target.position.z = 0.067
arm_group.set_pose_target(pose_target)
plan1 = arm_group.go()

hand_group.set_named_target("open")
plan2 = hand_group.go()

pose_target.position.z = 0.087
arm_group.set_pose_target(pose_target)
plan1 = arm_group.go()
    
#hand_group.set_named_target("close")
#plan2 = hand_group.go()
#rospy.sleep(5)
moveit_commander.roscpp_shutdown()
