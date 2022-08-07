#!/usr/bin/env python
import rospy
from std_msgs.msg import String
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
arm_group.set_named_target("home")
plan1 = arm_group.go()





def callback(data):
  if("blue" in data.data):
    

    listener = tf.TransformListener()

    listener.waitForTransform("/base_link", "/object_137", rospy.Time(), rospy.Duration(10.0))
    (trans, rot) = listener.lookupTransform('base_link', 'object_137', rospy.Time())
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
    rospy.sleep(2)
    pose_target.position.z = 0.002
    arm_group.set_pose_target(pose_target)
    plan1 = arm_group.go()
    rospy.sleep(2)

    hand_group.set_named_target("closed")
    plan2 = hand_group.go()
    rospy.sleep(2)

    pose_target.position.z = 0.067
    arm_group.set_pose_target(pose_target)
    plan1 = arm_group.go()
    rospy.sleep(2)

    listener.waitForTransform("/base_link", "/object_136", rospy.Time(), rospy.Duration(10.0))
    (trans2, rot2) = listener.lookupTransform('base_link', 'object_136', rospy.Time())

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
    rospy.sleep(2)	
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_rarm', anonymous=True)
    robot = moveit_commander.RobotCommander()

    arm_group = moveit_commander.MoveGroupCommander("ur5_arm")
    hand_group = moveit_commander.MoveGroupCommander("gripper")
    arm_group.set_named_target("home")
    plan1 = arm_group.go()


    rospy.Subscriber("speech_recognition/final_result", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

listener()
