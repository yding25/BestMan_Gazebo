#!/usr/bin/env python
import rospy, sys, numpy as np
import geometry_msgs.msg
import moveit_msgs.msg
from std_msgs.msg import UInt8
from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_srvs.srv import Empty

def gripper_on():
    rospy.wait_for_service('/ur5/vacuum_gripper0/on', timeout=60)
    rospy.wait_for_service('/ur5/vacuum_gripper1/on', timeout=60)
    rospy.wait_for_service('/ur5/vacuum_gripper2/on', timeout=60)
    rospy.wait_for_service('/ur5/vacuum_gripper3/on', timeout=60)
    rospy.wait_for_service('/ur5/vacuum_gripper4/on', timeout=60)
    rospy.wait_for_service('/ur5/vacuum_gripper5/on', timeout=60)
    rospy.wait_for_service('/ur5/vacuum_gripper6/on', timeout=60)
    rospy.wait_for_service('/ur5/vacuum_gripper7/on', timeout=60)
    rospy.wait_for_service('/ur5/vacuum_gripper8/on', timeout=60)
    try:
        resp0 = rospy.ServiceProxy('/ur5/vacuum_gripper0/on', Empty)
        resp1 = rospy.ServiceProxy('/ur5/vacuum_gripper1/on', Empty)
        resp2 = rospy.ServiceProxy('/ur5/vacuum_gripper2/on', Empty)
        resp3 = rospy.ServiceProxy('/ur5/vacuum_gripper3/on', Empty)
        resp4 = rospy.ServiceProxy('/ur5/vacuum_gripper4/on', Empty)
        resp5 = rospy.ServiceProxy('/ur5/vacuum_gripper5/on', Empty)
        resp6 = rospy.ServiceProxy('/ur5/vacuum_gripper6/on', Empty)
        resp7 = rospy.ServiceProxy('/ur5/vacuum_gripper7/on', Empty)
        resp8 = rospy.ServiceProxy('/ur5/vacuum_gripper8/on', Empty)
        return resp0(), resp1(), resp2(), resp3(), resp4(), resp5(), resp6(), resp7(), resp8()
    except rospy.ServiceException:
        print("Service call /ur5/vacuum_gripper/on failed")

def gripper_off():
    rospy.wait_for_service('/ur5/vacuum_gripper0/off', timeout=60)
    rospy.wait_for_service('/ur5/vacuum_gripper1/off', timeout=60)
    rospy.wait_for_service('/ur5/vacuum_gripper2/off', timeout=60)
    rospy.wait_for_service('/ur5/vacuum_gripper3/off', timeout=60)
    rospy.wait_for_service('/ur5/vacuum_gripper4/off', timeout=60)
    rospy.wait_for_service('/ur5/vacuum_gripper5/off', timeout=60)
    rospy.wait_for_service('/ur5/vacuum_gripper6/off', timeout=60)
    rospy.wait_for_service('/ur5/vacuum_gripper7/off', timeout=60)
    rospy.wait_for_service('/ur5/vacuum_gripper8/off', timeout=60)
    try:
        resp0 = rospy.ServiceProxy('/ur5/vacuum_gripper0/off', Empty)
        resp1 = rospy.ServiceProxy('/ur5/vacuum_gripper1/off', Empty)
        resp2 = rospy.ServiceProxy('/ur5/vacuum_gripper2/off', Empty)
        resp3 = rospy.ServiceProxy('/ur5/vacuum_gripper3/off', Empty)
        resp4 = rospy.ServiceProxy('/ur5/vacuum_gripper4/off', Empty)
        resp5 = rospy.ServiceProxy('/ur5/vacuum_gripper5/off', Empty)
        resp6 = rospy.ServiceProxy('/ur5/vacuum_gripper6/off', Empty)
        resp7 = rospy.ServiceProxy('/ur5/vacuum_gripper7/off', Empty)
        resp8 = rospy.ServiceProxy('/ur5/vacuum_gripper8/off', Empty)
        return resp0(), resp1(), resp2(), resp3(), resp4(), resp5(), resp6(), resp7(), resp8()
    except rospy.ServiceException:
        print("Service call /ur5/vacuum_gripper/off failed")

def trigger(msg):
    gripper_trigger = msg.data
    print('status of vacuum gripper: {}'.format(gripper_trigger))
    if gripper_trigger == True:
        gripper_on()
    else:
        gripper_off()

rospy.init_node("ur5_gripper", anonymous=False)
rospy.Subscriber('vacuum_gripper_trigger', Bool, trigger, queue_size=1)
rospy.spin()