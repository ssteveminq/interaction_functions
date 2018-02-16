#!/usr/bin/env python
import roslib
import rospy
import actionlib
import controller_manager_msgs.srv
import control_msgs.msg
import trajectory_msgs.msg
import roslib
import math
import sys
import rospy
import actionlib
import control_msgs.msg
import controller_manager_msgs.srv
import trajectory_msgs.msg
import geometry_msgs.msg
import controller_manager_msgs.srv
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import String
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from gaze_service.msg import GazingAction
from gaze_service.msg import GazingGoal

# BASE_STATE_TOPIC = "/hsrb/omni_base_controller/state"

class Gaze_manager(object):
    def __init__(self, wait=0.0):
        # initialize action client
        self.cli = actionlib.SimpleActionClient('gazing_action', GazingAction)
        self.cli.wait_for_server()
        # publisher for delvering command for base move
        rospy.Subscriber("/gaze_target", Marker, self.gaze_target_callback)
        # rospy.Subscriber("/global_pose",PoseStamped,self.Posecallback)
        # self.goal =GazingGoal(x_map=0,y_map=0)
        self.goal =GazingGoal()

    def sendactiongoal(self,desired_x_map, desired_y_map):
        self.goal.x_map=2.0
        self.goal.y_map=5.0
        
        self.cli.send_goal(self.goal)
        self.cli.wait_for_result()
    def listener(self,wait=0.0):
        # make sure the cont0roller is running
        # r=rospy.Rate(1)
        rospy.spin()            
        # rospy.sleep(0.1)
        # fill ROS message
    def gaze_target_callback(self, data):
        # rospy.loginfo(rospy.get_caller_id()+"I heard %d",data.data)
        print "gaze_callback"
        self.goal.x_map= data.pose.position.x
        self.goal.y_map= data.pose.position.y
        self.sendactiongoal(self.goal.x_map,self.goal.y_map)

        # print self.goal.x_map
        # print self.goal.y_map
        # self.cli.send_goal(self.goal)
        # self.cli.wait_for_result()
        # self.cli.get_result()
        
if __name__ == '__main__':
    rospy.init_node('gaze_action_client_test')
    gaze_manager = Gaze_manager(float(sys.argv[1]) if len(sys.argv) > 1 else 0.0)
    gaze_manager.listener()
    # gaze_manager.sendactiongoal()








































# rospy.init_node('test')

# # initialize action client
# cli = actionlib.SimpleActionClient('/hsrb/head_trajectory_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)

# # wait for the action server to establish connection
# cli.wait_for_server()

# # make sure the controller is running
# rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
# list_controllers = rospy.ServiceProxy('/hsrb/controller_manager/list_controllers', controller_manager_msgs.srv.ListControllers)
# running = False
# while running == False:
#     rospy.sleep(0.1)
#     for c in list_controllers().controller:
#         if c.name == 'head_trajectory_controller' and c.state == 'running':
#             running = True

# # fill ROS message
# goal = control_msgs.msg.FollowJointTrajectoryGoal()
# traj = trajectory_msgs.msg.JointTrajectory()
# traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
# p = trajectory_msgs.msg.JointTrajectoryPoint()
# p.positions = [0.5, 0.5]
# p.velocities = [0, 0]
# p.time_from_start = rospy.Time(3)
# traj.points = [p]
# goal.trajectory = traj

# # send message to the action server
# cli.send_goal(goal)

# # wait for the action server to complete the order
# cli.wait_for_result()
