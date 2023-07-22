#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# -----------------------------------------
# This code is to initialize the ur5 pose
# -----------------------------------------

from utils import Client

# -----------------------------------------
# intialize ur5 pose 
# -----------------------------------------
# init_joints = [0, -1.8, 1.4, -1.18, -1.56, 0]
init_joints = [0, -2.2, 2.2, -1.57, -1.57, 0]

# -----------------------------------------
# instantiate
# -----------------------------------------
demo = Client()

# -----------------------------------------
# stop segbot's movement
# -----------------------------------------
demo.stop()

# -----------------------------------------
# get joint status (pose for avoidance) 
# -----------------------------------------
demo.plan_joint_goal(init_joints[0], init_joints[1], init_joints[2], init_joints[3], init_joints[4], init_joints[5])
print('pose for avoidance is done!')