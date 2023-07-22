#!/usr/bin/env python
# -----------------------------------------
# This code is to pick and place an object
# -----------------------------------------

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
import actionlib
from math import pi, acos, tan, cos, sin, atan2
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from control_msgs.msg import *
from trajectory_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point

from utils import Client

# -----------------------------------------
# intialize ur5 pose
# -----------------------------------------
# init_joints = [0, -1.8, 1.4, -1.18, -1.56, 0]
init_joints = [0, -2.2, 2.2, -1.57, -1.57, 0]

# -----------------------------------------
# robot position before grasping
# -----------------------------------------
base_position = [6.6, 0]
base_orientation = [0.0, 0.0, 0.1, 1.0]

# -----------------------------------------
# robot position before putting down
# -----------------------------------------
place_loc = [-2.4000260221, 5.27721117742]
place_orien = [0, 0, -0.705757194679, -0.708453782101]

# -----------------------------------------
# object pose for putting down
# -----------------------------------------
place_ee_loc = [-2.4, 5.8]

# -----------------------------------------
# instantiate
# -----------------------------------------
demo = Client()

# -----------------------------------------
# get joint status (pose for avoidance) 
# -----------------------------------------
demo.plan_joint_goal(init_joints[0],init_joints[1],init_joints[2], init_joints[3],init_joints[4], init_joints[5])
rospy.sleep(1.0)
print('pose for avoidance is done!')
print('-'*30)

# -----------------------------------------
# ungrasp
# -----------------------------------------
# demo.ungrasp()
# rospy.sleep(1.0)
# print('ungrasp is done!')
# print('-'*30)

# -----------------------------------------
# move to a position close to target object
# -----------------------------------------
demo.navigate_to_position(base_position, base_orientation)
rospy.sleep(1.0)
print('move to a position close to target object is done!')
print('-'*30)

# -----------------------------------------
# compute pose for grasping (Xiaohan's method)
# -----------------------------------------
object_loc, object_orien = demo.get_model_pose('box_blue')
pose_goal = demo.compute_pose_for_grasp_alternative(object_loc, object_orien)
rospy.sleep(1.0)

# -----------------------------------------
# compute pose for grasping (Yan's method)
# -----------------------------------------
# object_loc, object_orien = demo.get_model_pose('box_blue')
# pose_goal = demo.compute_pose_for_grasp(object_loc, object_orien)
# rospy.sleep(1.0)
print('pose_goal: {}'.format(pose_goal))

# -----------------------------------------
# reach goal
# -----------------------------------------
x_init = pose_goal.position.x
y_init = pose_goal.position.y
z_init = pose_goal.position.z

# x, y, z = demo.point1(x_init, y_init, z_init)
# print('x:{} y:{} z:{}'.format(x, y, z))
# demo.plan_position_goal(x, y, z)
# rospy.sleep(1.0)

# x, y, z = demo.point2(x, y, z)
# print('x:{} y:{} z:{}'.format(x, y, z))
# demo.plan_position_goal(x, y, z)
# rospy.sleep(1.0)

# x, y, z = demo.point3(x, y, z)
# print('x:{} y:{} z:{}'.format(x, y, z))
# demo.plan_position_goal(x, y, z)
# rospy.sleep(1.0)

demo.plan_position_goal(x_init, y_init, z_init)
rospy.sleep(1.0)
print('compute pose for grasping & reach goal is done!')
print('-'*30)

# -----------------------------------------
# grasp
# -----------------------------------------
demo.grasp(0.38)
rospy.sleep(2.0)
print('grasp is done!')
print('-'*30)

# -----------------------------------------
# get joint status (pose for avoidance) 
# -----------------------------------------
demo.plan_joint_goal(init_joints[0], init_joints[1], init_joints[2], init_joints[3], init_joints[4], init_joints[5])
rospy.sleep(1.0)
print('pose for avoidance is done!')
print('-'*30)