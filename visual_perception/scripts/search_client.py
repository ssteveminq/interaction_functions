#!/usr/bin/python
# -*- coding: utf-8 -*-
import hsrb_interface
from hsrb_interface import geometry
import rospy
import sys
import tf
import actionlib
from geometry_msgs.msg import *
from visual_perception.msg import *
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Point, Quaternion, PointStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

def mains():
    # Initialize
    client = actionlib.SimpleActionClient('search_server',visual_perception.msg.SearchAction)
    rospy.loginfo("wait_server")
    client.wait_for_server()
    rospy.loginfo("requesting_action_server")
    goal = visual_perception.msg.SearchGoal()

    # goal.target=pose

    client.send_goal(goal)
    # rospy.loginfo("start action")
    client.wait_for_result(rospy.Duration(20.0))
    # rospy.loginfo("result %s", result)
    result = client.get_result()
    print result
        
if __name__ == '__main__':
    rospy.init_node('search_client')
    mains()
