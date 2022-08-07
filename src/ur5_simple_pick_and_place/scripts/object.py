#! /usr/bin/env python
import os
import sys
import rospy
import tf

def get_pose_coord():

  rospy.init_node('ur5_arm', anonymous=True)
  listener = tf.TransformListener()

  listener.waitForTransform("/base_link", "object_134", rospy.Time(), rospy.Duration(8.0))
  (trans, rot) = listener.lookupTransform('base_link', 'object_134', rospy.Time())

  goal_coord = [trans[0], trans[1], trans[2]] 
  goal_orient = [rot[0], rot[1], rot[2], rot[3]]
  print(goal_coord)
  print(goal_orient)
  
get_pose_coord()