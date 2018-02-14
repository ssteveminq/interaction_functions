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
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import String
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

BASE_STATE_TOPIC = "/hsrb/omni_base_controller/state"




class Gaze_manager(object):
    def __init__(self, wait=0.0):
        # initialize action client
        self.cli = actionlib.SimpleActionClient('gazing_action', gaze_service.msg.GazingAction)
        # publisher for delvering command for base move
        # self.vel_pub = rospy.Publisher('/hsrb/command_velocity', geometry_msgs.msg.Twist, queue_size=10)
        # Subscriber for receiving GUI command 
        # rospy.Subscriber("/cmd", Int8, self.intcallback)
        rospy.Subscriber("/human_boxes", , self.intcallback)
        rospy.Subscriber("/hsrb/odom", Odometry, self.odometryInput)
        # rospy.Subscriber("/global_pose",PoseStamped,self.Posecallback)
        
        rospy.Subscriber(BASE_STATE_TOPIC, JointTrajectoryControllerState,self.state_callback, queue_size=1)
        # message initialization
        self.tw = geometry_msgs.msg.Twist()
        #
        while self.vel_pub.get_num_connections() == 0:
                rospy.sleep(0.2)
        # Initialization of goal & trajectory
        self.goal = control_msgs.msg.FollowJointTrajectoryGoal()
        self.traj = trajectory_msgs.msg.JointTrajectory()
        self.traj.joint_names = ["odom_x", "odom_y", "odom_t"]
        self.p = trajectory_msgs.msg.JointTrajectoryPoint()
        self.p.positions = [0, 0, 0.0]
        self.p.velocities = [0, 0, 0]
        self.p.time_from_start = rospy.Time(5)
        self.traj.points = [self.p]
        self.goal.trajectory = self.traj
        self.cli.wait_for_server()
        self.robot_pos_x=0.0
        self.robot_pos_y=0.0
        self.robot_pos_z=0.0
        self.robot_ori_z=0.0
        self.last_robot_theta=0.0
        self.global_offset_x=0.0
        self.global_offset_y=0.0
        self.global_offset_origi_z=0.0

    def state_callback(self,data):
        self.last_robot_x = data.actual.positions[0];
        self.last_robot_y = data.actual.positions[1];
        self.last_robot_theta = data.actual.positions[2];
    # def Posecallback(self,data):
    #     self.global_offset_x=data.pose.position.x;
    #     self.global_offset_y=data.pose.position.y;
    #     self.global_offset_origi_z=data.pose.orientation.z;
    #     self.global_offset_origi_z=math.asin(self.global_offset_origi_z)*2
    def odometryInput(self, data):
        self.robot_pos_x = data.pose.pose.position.x+self.global_offset_x
        self.robot_pos_y = data.pose.pose.position.y+self.global_offset_y
        self.robot_pos_z = data.pose.pose.position.z
        self.robot_ori_z = data.pose.pose.orientation.z
        self.robot_ori_z = math.asin(self.robot_ori_z)*2+self.global_offset_origi_z
        # rospy.loginfo(rospy.get_caller_id()+"x : %.3lf , y : %.3lf , z : %.3lf",self.robot_pos_x,self.robot_pos_y,self.robot_ori_z)


    def listener(self,wait=0.0):
        # make sure the controller is running
        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        self.list_controllers = rospy.ServiceProxy('/hsrb/controller_manager/list_controllers', controller_manager_msgs.srv.ListControllers)
        running = False
        while running == False:
            rospy.sleep(0.1)
            for c in self.list_controllers().controller:
                if c.name == 'omni_base_controller' and c.state == 'running':
                    running = True
        rospy.spin()            
        # fill ROS message
    def intcallback(self, data):
        rospy.loginfo(rospy.get_caller_id()+"I heard %d",data.data)
        self.pre_ori_z=self.robot_ori_z
         # command = data.data
        if data.data == 1:                       #Going home
            # self.tw.linear.x =0
            # self.tw.angular.z = 0.6
            # self.vel_pub.publish(self.tw)
            # self.p.positions = [self.robot_pos_x,self.robot_pos_y,self.robot_ori_z+math.pi/4]
            self.p.positions = [self.robot_pos_x,self.robot_pos_y,self.last_robot_theta+math.pi/4]
            self.p.velocities =[0,0,0]
            self.traj.points = [self.p]
            self.goal.trajectory = self.traj
            self.p.time_from_start = rospy.Time(3)
            self.cli.send_goal(self.goal)
            self.cli.wait_for_result()
        elif data.data == 2:
            # self.p.positions = [self.robot_pos_x+1,self.robot_pos_y,self.last_robot_theta]
            # self.p.velocities =[0,0,0]
            # self.traj.points = [self.p]
            # self.goal.trajectory = self.traj
            # self.p.time_from_start = rospy.Time(3)
            # self.cli.send_goal(self.goal)
            # self.cli.wait_for_result()
            self.tw.linear.x = 0.2
            self.vel_pub.publish(self.tw)
        elif data.data == 3:
            self.p.positions = [self.robot_pos_x,self.robot_pos_y,self.last_robot_theta-math.pi/4]
            self.traj.points = [self.p]
            self.goal.trajectory = self.traj
            self.p.time_from_start = rospy.Time(3)
            self.cli.send_goal(self.goal)
            self.cli.wait_for_result()
            # self.tw.linear.x =0
            # self.tw.angular.z = -0.6
            # self.vel_pub.publish(self.tw)
        elif data.data == 4:                     #Going home
            self.tw = geometry_msgs.msg.Twist()
            self.tw.linear.x = -0.2
            self.vel_pub.publish(self.tw)
        elif data.data == 5:
            self.tw = geometry_msgs.msg.Twist()
            self.tw.angular.z=0.5;
            self.vel_pub.publish(self.tw)
        elif data.data == 6:
            self.tw = geometry_msgs.msg.Twist()
            self.tw.angular.z=-0.5;
            self.vel_pub.publish(self.tw)
        elif data.data == 7:
            n=8
            for counter in range(1,n):
                self.tw = geometry_msgs.msg.Twist()
                self.tw.linear.x=0.1;
                self.vel_pub.publish(self.tw)
                rospy.sleep(0.8)
        elif data.data == 8:
            n=8
            for counter in range(1,n):
                self.tw = geometry_msgs.msg.Twist()
                self.tw.linear.x=-0.1;
                self.vel_pub.publish(self.tw)
                rospy.sleep(0.8)
        elif data.data == 9:
            self.p.positions = [self.robot_pos_x,self.robot_pos_y,self.last_robot_theta]
            self.p.velocities =[0,0,0]
            self.traj.points = [self.p]
            self.goal.trajectory = self.traj
            self.p.time_from_start = rospy.Time(3)
            self.cli.send_goal(self.goal)
            self.cli.wait_for_result()
        elif data.data == 10:
            self.goal.trajectory = self.traj
            self.p.time_from_start = rospy.Time(3)
            self.cli.send_goal(self.goal)
            self.cli.wait_for_result()
        else:
            self.tw.angular.z = 0.0
        
        



# # send message to the action server
# cli.send_goal(goal)

# # wait for the action server to complete the order
# cli.wait_for_result()
if __name__ == '__main__':
    rospy.init_node('gaze managing test')
    gaze_manager = Gaze_manager(float(sys.argv[1]) if len(sys.argv) > 1 else 0.0)
	gaze_manager.listener()















































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
