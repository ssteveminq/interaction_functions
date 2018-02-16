#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Int8
import std_msgs.msg
import rospy
import math

topic = 'gaze_target'
topic_array = 'human_boxes'
publisher = rospy.Publisher(topic, Marker,queue_size=10)
# array_publisher = rospy.Publisher(topic_array, MarkerArray,queue_size=10)
pub = rospy.Publisher('/Int_cmd_trackhuman', Int8, queue_size=10)

rospy.init_node('register')

markerArray = MarkerArray()

count = 0
MARKERS_MAX = 100

while not rospy.is_shutdown():

   # del markerArray.markers[:]
   
   marker = Marker()
   marker.header.frame_id = "/map"
   marker.type = marker.SPHERE
   marker.action = marker.ADD
   marker.scale.x = 0.5
   marker.scale.y = 0.5
   marker.scale.z = 0.5
   marker.color.a = 1.0
   marker.color.r = 1.0
   marker.color.g = 0.0
   marker.color.b = 0.0
   marker.pose.orientation.w = 1.0
   
   # marker.pose.position.x = 2.1
   # marker.pose.position.y =10.3
   # marker.pose.position.z = 1
   marker.id=0

   #moving human
   marker.pose.position.x =0.2+0.005*count+1.5*math.cos(count/300.0)
   marker.pose.position.y = -2.3+0.002*count+0.5*math.sin(count/300.0)
   marker.pose.position.z = 1

   # marker.pose.position.y = 2.7+0.2*math.sin(count/100.0)
   # marker.pose.position.y = +0.2*math.sin(count / 40.0) 
   
   # marker.pose.position.x = 3.0
   # marker.pose.position.y = 0.5
   # marker.pose.position.z = 1

   # int_msg=std_msgs.msg.Int8()
   # int_msg.data=1

   # We add the new marker to the MarkerArray, removing the oldest
   # marker from it when necessary
   # if(count > MARKERS_MAX):
   #     markerArray.markers.pop(0)

   publisher.publish(marker)

   count += 1.0
   # if count>100:
       # count=0

   rospy.sleep(1.0)
