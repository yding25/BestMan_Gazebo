#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# -----------------------------------------
# This code is to control Segbot & UR5e
# -----------------------------------------

import roslib; roslib.load_manifest('ur_driver')
from control_msgs.msg import *
from trajectory_msgs.msg import *
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
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import *
import tf
import numpy as np
import random
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_srvs.srv import Empty


'''
function list:
1, display_trajectory_rviz
2, plan_joint_goal (*)
3, plan_joint_goal_client
4, all_close
5, ungrasp (*)
6, grasp (*)
7, navigate_to_position (*)
8, compute_pose_for_grasp (*)
9, compute_pose_for_putdown (*)
10, plan_pose_goal (*)
11, plan_position_goal (*)
12, wait_for_state_update
13, add_fixed_obstacle
14, add_movable_obstacle
15, remove_movable_obstacle
16, remove_fixed_obstacle
17, get_transformation_map_tool0
18, get_joint_values
19, get_segbot_pose
20, set_model_pose
21, compute_pose_for_grasp_alternative
22, quaternion_multiply
23, get_transformation_map_odom
24, quaternion_to_euler
'''

class Client(object):
    def __init__(self):
        # -----------------------------------------
        # initialize moveit_commander and rospy
        # -----------------------------------------
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("demo", anonymous=True, disable_signals=True)

        # -----------------------------------------        
        # instantiate RobotCommander
        # -----------------------------------------
        self.robot = moveit_commander.RobotCommander()

        # -----------------------------------------        
        # instantiate PlanningSceneInterface
        # -----------------------------------------
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # -----------------------------------------        
        # instantiate MoveGroupCommander
        # -----------------------------------------
        group_name = 'manipulator'  # group_name can be find in ur5_moveit_config/config/ur5.srdf
        self.move_group_commander = moveit_commander.MoveGroupCommander(group_name)

        # -----------------------------------------        
        # control speed
        # -----------------------------------------
        self.move_group_commander.set_max_velocity_scaling_factor(0.05)
        self.move_group_commander.set_max_acceleration_scaling_factor(0.05)

        # -----------------------------------------        
        # allow replanning
        # -----------------------------------------
        self.move_group_commander.allow_replanning(True)
        self.move_group_commander.set_planning_time(500.0)
        self.move_group_commander.set_goal_position_tolerance(0.02)
        self.move_group_commander.set_goal_orientation_tolerance(0.02)
    
        # -----------------------------------------
        # connect to client1: arm_controller/follow_joint_trajectory
        # -----------------------------------------
        try:
            self.client1 = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            print ("Waiting for client1")
            self.client1.wait_for_server()
            print ("Connected to client1")
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

        # -----------------------------------------
        # connect to client3: move_base
        # -----------------------------------------
        try:
            self.client3 = actionlib.SimpleActionClient("move_base", MoveBaseAction)
            print ("Waiting for client3")
            self.client3.wait_for_server()
            print ("Connected to client3")
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise
            
        # -----------------------------------------
        # create listener
        # -----------------------------------------
        self.listener = tf.TransformListener()
        print('-'*30)


    def display_trajectory_rviz(self, plan):
        # -----------------------------------------
        # visualize the trajectory 
        # -----------------------------------------
        
        # create a publisher
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        # record trajectory
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)

        # publish trajectory
        display_trajectory_publisher.publish(display_trajectory)


    def plan_joint_goal(self, joint0, joint1, joint2, joint3, joint4, joint5):
        # ------------------------------
        # set joint goal
        # ------------------------------
        rospy.sleep(1.0) # debug

        joint_goal = self.move_group_commander.get_current_joint_values()
        joint_goal[0] = joint0
        joint_goal[1] = joint1
        joint_goal[2] = joint2
        joint_goal[3] = joint3
        joint_goal[4] = joint4
        joint_goal[5] = joint5
        '''
        The go command can be called with joint values, poses, or without any
        parameters if you have already set the pose or joint target for the group
        '''
        self.move_group_commander.go(joint_goal, wait=True)
        
        # ------------------------------
        # stop and ensure that there is no residual movement
        # ------------------------------
        self.move_group_commander.stop()

        '''
        It is always good to clear your targets after planning with poses.
        Note: there is no equivalent function for clear_joint_value_targets()
        '''
        self.move_group_commander.clear_pose_targets()


    def plan_joint_goal_client(self, joint0, joint1, joint2, joint3, joint4, joint5):
        '''
        different from "plan_joint_goal", this function use client to send goal commands
        '''
        # ------------------------------
        # process joint goal
        # ------------------------------
        # JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        JOINT_NAMES = ['ur5_shoulder_pan_joint', 'ur5_shoulder_lift_joint', 'ur5_elbow_joint', 'ur5_wrist_1_joint', 'ur5_wrist_2_joint', 'ur5_wrist_3_joint']
        joint_goal = FollowJointTrajectoryGoal()
        joint_goal.trajectory = JointTrajectory()
        joint_goal.trajectory.joint_names = JOINT_NAMES
        joint_goal.trajectory.points = [JointTrajectoryPoint(positions=[joint0, joint1, joint2, joint3, joint4, joint5], velocities=[0]*6, time_from_start=rospy.Duration(4.0))]

        # ------------------------------
        # send joint goal to client
        # ------------------------------
        self.client1.send_goal(joint_goal)
        try:
            self.client1.wait_for_result(rospy.Duration(60))
        except KeyboardInterrupt:
            self.client1.cancel_goal()
            print('error! cannot send joint goal to client1')
            raise


    def all_close(self, goal, actual, tolerance):
        # ------------------------------
        # check if a list of values are within a tolerance of their counterparts in another list
        # goal: a list of floats, a Pose or a PoseStamped
        # actual: a list of floats, a Pose or a PoseStampedbase_position
        # tolerance: a float
        # ------------------------------
        all_equal = True
        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    return False
        elif type(goal) is geometry_msgs.msg.PoseStamped:
            return all_close(goal.pose, actual.pose, tolerance)
        elif type(goal) is geometry_msgs.msg.Pose:
            return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
        return True

    def ungrasp(self):
        try:
            pub = rospy.Publisher('vacuum_gripper_trigger', Bool, queue_size=1)
            while pub.get_num_connections() < 1:
                pass
            pub.publish(Bool(False))
        except rospy.ROSInterruptException:
            pass

    def grasp(self):
        try:
            pub = rospy.Publisher('vacuum_gripper_trigger', Bool, queue_size=1)
            while pub.get_num_connections() < 1:
                pass
            pub.publish(Bool(True))
        except rospy.ROSInterruptException:
            pass

    def navigate_to_position(self, position, orientation):
        # ------------------------------
        # process position
        # ------------------------------
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(position[0], position[1], 0)
        goal.target_pose.pose.orientation.x = orientation[0]
        goal.target_pose.pose.orientation.y = orientation[1]
        goal.target_pose.pose.orientation.z = orientation[2]
        goal.target_pose.pose.orientation.w = orientation[3]
        
        # ------------------------------
        # send goal to client
        # ------------------------------
        self.client3.send_goal(goal)
        try:
            self.client3.wait_for_result(rospy.Duration(60))
        except KeyboardInterrupt:
            self.client3.cancel_goal()
            print('error! cannot send goal to client3')
            raise
    

    def quaternion_multiply(self, quaternion1, quaternion0):
        w0, x0, y0, z0 = quaternion0
        w1, x1, y1, z1 = quaternion1
        return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                        x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                        -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                        x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)


    def compute_pose_for_grasp(self, object_position, object_orientation):
        while not rospy.is_shutdown():
            try:
                (transformed_position, transformed_oritention) = self.listener.lookupTransform('/odom', '/ur5_base_link', rospy.Time(0))
                if transformed_position:
                    break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        print('object_position: {}'.format(object_position))
        print('object_orientation: {}'.format(object_orientation))
        print('transformed_position: {}'.format(transformed_position))
        print('transformed_oritention: {}'.format(transformed_oritention))

        # -----------------------------------------
        # get gripper pose
        # -----------------------------------------
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = object_position[0] - transformed_position[0] - 0.05/2.0
        pose_goal.position.y = object_position[1] - transformed_position[1] + 0.05/2.0
        # pose_goal.position.z = object_position[2] - transformed_position[2] + 1.5 + 0.05 / 2.0 # 0.15 is gripper height and 0.1 is object height
        pose_goal.position.z = 1.10

        quaternion = R.from_euler('zyx',[0, np.pi/2, 0]).as_quat() # rotate y 90 degrees
        # temp = quaternion_multiply(transformed_oritention, [object_orientation[0], object_orientation[1], object_orientation[2], object_orientation[3]])
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]
        return pose_goal


    def compute_pose_for_putdown(self, table_position):
        # -----------------------------------------
        # robot pose
        # -----------------------------------------
        segbot_pose = rospy.wait_for_message('/odom', Odometry).pose.pose
        position_x = segbot_pose.position.x
        position_y = segbot_pose.position.y

        # -----------------------------------------
        # convert quaternion to yaw value
        # -----------------------------------------
        w = rospy.wait_for_message('/odom', Odometry).pose.pose.orientation.w
        x = rospy.wait_for_message('/odom', Odometry).pose.pose.orientation.x
        y = rospy.wait_for_message('/odom', Odometry).pose.pose.orientation.y
        z = rospy.wait_for_message('/odom', Odometry).pose.pose.orientation.z
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        theta = atan2(siny_cosp, cosy_cosp)

        # -----------------------------------------
        # object pose
        # -----------------------------------------
        dx = table_position[0]
        dy = table_position[1]
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = ((dy-position_y)*tan(theta) - (position_x-dx))/(cos(theta) + sin(theta)*tan(theta))
        pose_goal.position.y = (dy - position_y - (dx - position_x)*tan(theta))/(sin(theta)*tan(theta) + cos(theta))
        pose_goal.position.z = 1.3
        current_pose = self.move_group_commander.get_current_pose().pose
        pose_goal.orientation = current_pose.orientation
        return pose_goal


    def plan_pose_goal(self, pose_goal):
        # ------------------------------
        # plan to achieve pose goal
        # ------------------------------
        self.move_group_commander.set_pose_target(pose_goal)
        plan = self.move_group_commander.go(wait=True)
        
        # ------------------------------
        # stop and ensure that there is no residual movement
        # ------------------------------
        self.move_group_commander.stop()

        '''
        It is always good to clear your targets after planning with poses.
        Note: there is no equivalent function for clear_joint_value_targets()
        '''
        self.move_group_commander.clear_pose_targets()


    def plan_position_goal(self, x, y, z):
        waypoints = []
        # ------------------------------
        # generate a goal position
        # ------------------------------
        wpose = self.move_group_commander.get_current_pose().pose
        print('current pose:{}'.format(wpose))
        print('-'*30)

        wpose.position.x = x
        wpose.position.y = y
        wpose.position.z = z
        waypoints.append(copy.deepcopy(wpose))

        '''
        We want the Cartesian path to be interpolated at a resolution of 1 cm
        which is why we will specify 0.01 as the eef_step in Cartesian
        translation.  We will disable the jump threshold by setting it to 0.0,
        ignoring the check for infeasible jumps in joint space, which is sufficient
        for this tutorial.
        '''
        # compute_cartesian_path(eef_step, waypoints to follow, jump_threshold)
        (plan, fraction) = self.move_group_commander.compute_cartesian_path(waypoints, 0.01, 0.0)
        velocity_scaling_factor = 0.05
        plan = self.move_group_commander.retime_trajectory(self.robot.get_current_state(), plan, velocity_scaling_factor)
        '''
        Use execute if you would like the robot to follow
        the plan that has already been computed:
        '''
        self.move_group_commander.execute(plan, wait=True)

        # ------------------------------
        # stop and ensure that there is no residual movement
        # ------------------------------
        self.move_group_commander.stop()

        '''
        It is always good to clear your targets after planning with poses.
        Note: there is no equivalent function for clear_joint_value_targets()
        '''
        self.move_group_commander.clear_pose_targets()


    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([self.box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = self.box_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(1.0)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False


    def add_fixed_obstacle(self, obstacle_name, frame, obstacle_size, obstsacle_position, timeout=4):
        # obstacle pose
        obstacle_pose = PoseStamped()
        obstacle_pose.header.frame_id = frame
        obstacle_pose.pose.position.x = obstsacle_position[0]
        obstacle_pose.pose.position.y = obstsacle_position[1]
        obstacle_pose.pose.position.z = obstsacle_position[2]
        obstacle_pose.pose.orientation.w = 1.0

        # add obstacle
        self.scene.add_box(obstacle_name, obstacle_pose, obstacle_size)
        rospy.sleep(1.0)
        

    def add_movable_obstacle(self, obstacle_name, frame, obstacle_size, obstsacle_position, timeout=4):
        # obstacle pose
        obstacle_pose = PoseStamped()
        obstacle_pose.header.frame_id = frame
        obstacle_pose.pose.position.x = obstsacle_position[0]
        obstacle_pose.pose.position.y = obstsacle_position[1]
        obstacle_pose.pose.position.z = obstsacle_position[2]
        obstacle_pose.pose.orientation.w = 1.0

        # add obstacle
        self.scene.attach_box(frame, obstacle_name, obstacle_pose, obstacle_size)
        rospy.sleep(1.0)


    def remove_movable_obstacle(self, obstacle_name, frame):
        self.scene.remove_attached_object(frame, obstacle_name)
        rospy.sleep(1.0)


    def remove_fixed_obstacle(self, obstacle_name):
        self.scene.remove_world_object(obstacle_name)
        rospy.sleep(1.0)


    def get_joint_values(self):
        joints = self.move_group_commander.get_current_joint_values()
        print('joints:{}'.format(joints))
        return joints


    def get_segbot_pose(self):
        segbot_pose = rospy.wait_for_message('/odom', Odometry).pose.pose
        position_x = segbot_pose.position.x
        position_y = segbot_pose.position.y
        position_z = segbot_pose.position.z
        orientation_x = segbot_pose.orientation.x
        orientation_y = segbot_pose.orientation.y
        orientation_z = segbot_pose.orientation.z
        orientation_w = segbot_pose.orientation.w
        return position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w


    def get_model_pose(self, model_name):
        get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        model = GetModelStateRequest()
        model.model_name = model_name
        model_pose = get_state_service(model).pose
        # get model pose
        position_x = model_pose.position.x
        position_y = model_pose.position.y
        position_z = model_pose.position.z
        orientation_x = model_pose.orientation.x
        orientation_y = model_pose.orientation.y
        orientation_z = model_pose.orientation.z
        orientation_w = model_pose.orientation.w
        return [position_x, position_y, position_z], [orientation_x, orientation_y, orientation_z, orientation_w]


    def set_model_pose(self, model_name, position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w):
        rospy.wait_for_service('/gazebo/set_model_state')
        set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        model_pose = SetModelStateRequest()
        # set new model pose
        # model_pose.model_state.reference_frame = 'map'
        model_pose.model_state.reference_frame = 'odom'
        model_pose.model_state.model_name = model_name
        model_pose.model_state.pose.position.x = position_x + 0.1
        model_pose.model_state.pose.position.y = position_y + 0.1
        model_pose.model_state.pose.position.z = position_z + 0.1
        model_pose.model_state.pose.orientation.w = orientation_w
        model_pose.model_state.pose.orientation.x = orientation_x
        model_pose.model_state.pose.orientation.y = orientation_y
        model_pose.model_state.pose.orientation.z = orientation_z
        set_state_service(model_pose)


    def compute_pose_for_grasp_alternative(self, object_position, object_orientation):
        # -----------------------------------------
        # get segbot pose
        # -----------------------------------------
        segbot_pose = rospy.wait_for_message('/odom', Odometry).pose.pose
        position_x = segbot_pose.position.x
        position_y = segbot_pose.position.y
        orientation_x = segbot_pose.orientation.x
        orientation_y = segbot_pose.orientation.y
        orientation_z = segbot_pose.orientation.z
        orientation_w = segbot_pose.orientation.w
        
        # -----------------------------------------
        # target object position
        # -----------------------------------------
        object_x = object_position[0]
        object_y = object_position[1]

        # -----------------------------------------
        # convert quaternion to yaw value
        # -----------------------------------------
        siny_cosp = 2 * (orientation_w * orientation_z + orientation_x * orientation_y)
        cosy_cosp = 1 - 2 * (orientation_y * orientation_y + orientation_z * orientation_z)
        theta = atan2(siny_cosp, cosy_cosp)
        
        # -----------------------------------------
        # rotate the gripper so that it can grasp
        # -----------------------------------------
        roll = 0
        pitch = pi/2
        yaw = 0

        # -----------------------------------------
        # convert roll pitch yaw into quaternion
        # -----------------------------------------
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        # -----------------------------------------
        # get gripper pose
        # -----------------------------------------
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = ((object_y - position_y) * tan(theta) - (position_x - object_x))/(cos(theta) + sin(theta) * tan(theta))
        pose_goal.position.y = (object_y - position_y - (object_x - position_x) * tan(theta))/(sin(theta) * tan(theta) + cos(theta))
        pose_goal.position.z = 1.08
        pose_goal.orientation.w = cr * cp * cy + sr * sp * sy
        pose_goal.orientation.x = sr * cp * cy - cr * sp * sy
        pose_goal.orientation.y = cr * sp * cy + sr * cp * sy
        pose_goal.orientation.z = cr * cp * sy - sr * sp * cy 
        print('pose_goal in compute_pose_for_grasp_alternative:{}'.format(pose_goal))
        return pose_goal


    def point1(self, x, y, z):
        x = x
        y = y
        z = 2
        return x, y, z


    def point2(self, x, y, z):
        x = x
        y = y
        z = 1.8
        return x, y, z
    

    def point3(self, x, y, z):
        x = x
        y = y
        z = 1.5
        return x, y, z

    
    def stop(self):
        # ------------------------------
        # stop and ensure that there is no residual movement
        # ------------------------------
        try: 
            self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
            self.command = Twist()
            self.pub_vel.publish(self.command)
            rospy.sleep(1.0)
        except rospy.ROSInterruptException:
            pass