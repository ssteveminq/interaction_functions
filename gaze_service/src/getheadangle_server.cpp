#include "ros/ros.h"
#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <string>
#include <map>
#include <sstream>
#include <signal.h>
#include <boost/thread/thread.hpp>
// #include <stdint.h>

#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/server/simple_action_server.h>
#include <gaze_service/GetHeadAngleAction.h>

#define LASER_RANGE 5.5
#define one_step_angle 0.4
#define TIME_INTERVAL_FIND_HUMAN 1.5
#define TIME_INTERVAL_GAZE 1.5
#define TIME_NORMAL_INTERVAL_GAZE 1.2

using namespace Eigen;

class HeadAngle_Manager{

public:
  explicit HeadAngle_Manager(std::string name):
      as_(nh_, name, boost::bind(&HeadAngle_Manager::executeCB, this,_1), false),
      action_name_(name),
      IsActivated(false),
      time_found_human(0.0),
      Pre_IsHuman(false),
      IsHuman(false),
      desired_target(0.0),
      cmd_flag(0),
      IsRobotMoving(false),
      last_target_direction(0){
  
   global_pose.resize(3,0.0);
   pre_global_pose.resize(3,0.0);
   gaze_target_pose.resize(3,0.0);
   Head_Pos.resize(2,0.0);
   nav_target.resize(3,0.0);
   time_gaze_published=0.0;

   as_.registerPreemptCallback(boost::bind(&HeadAngle_Manager::preemptCB, this));
   as_.start();
  
  }
  ~HeadAngle_Manager(){}

protected:
  ros::NodeHandle nh_;
  std::string action_name_;
  actionlib::SimpleActionServer<gaze_service::GetHeadAngleAction> as_;
  gaze_service::GetHeadAngleFeedback feedback_;
  gaze_service::GetHeadAngleResult result_;

public:
  ros::Publisher Desired_joint_pub; 
  ros::Publisher navTarget_pub; 
  tf::TransformListener     listener;
  geometry_msgs::PoseArray human_poses;
  geometry_msgs::Pose gaze_target;

  std::vector<double> global_pose;
  std::vector<double> pre_global_pose;
  std::vector<double> gaze_target_pose;
  std::vector<double> Head_Pos; 
  std::vector<double> temp_human_target; 
  std::vector<double> nav_target; 
  std::vector< std::vector< double > > Cur_leg_human;

  double desired_target;
  double time_found_human;
  double time_gaze_published;
  int last_target_direction; // left :1 . right :-1
  int cmd_flag;
  bool IsRobotMoving;
  bool IsHuman;
  bool Pre_IsHuman;
  bool IsActivated;

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void Publish_gazetarget(float _x, float _y)
  {
      ROS_INFO("x : %.3lf , y : %.3lf", _x,_y);
  }
 
void targetpose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){

    gaze_target=msg->pose;
    ROS_INFO("target pose received");
    gaze_target_pose.resize(3,0.0);
    gaze_target_pose[0]=gaze_target.position.x;
    gaze_target_pose[1]=gaze_target.position.y;
    gaze_target_pose[2]=0.75;
}

void globalpose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){

   global_pose.resize(3);

   tf::StampedTransform baselinktransform;
   listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
   listener.lookupTransform("map", "base_link", ros::Time(0), baselinktransform);
   double yaw_tf =   tf::getYaw(baselinktransform.getRotation()); 

   global_pose[0]=msg->pose.position.x;
   global_pose[1]=msg->pose.position.y;
   global_pose[2]=yaw_tf;
   //ROS_INFO("global pose x: %.2lf y: %.2lf z: %.2lf",global_pose[0],global_pose[1],global_pose[2]);

   double move_distance =0.0;
    for(size_t idx(0);idx<global_pose.size();idx++)
        move_distance+=pow((pre_global_pose[idx]-global_pose[idx]),2);
    move_distance=sqrt(move_distance);

    //ROS_INFO("robot is moving : %.3lf", move_distance);

    if(move_distance>0.007)
        IsRobotMoving=true;
    else
        IsRobotMoving=false;

    //ROS_INFO("robot is moving : %d", IsRobotMoving);

     for(size_t idx(0);idx<global_pose.size();idx++)
        pre_global_pose[idx]=global_pose[idx];
}

bool Comparetwoposes(std::vector<double> pos,std::vector<double> pos2, double criterion)
{
  //return true if there are in criterion distance 
  double temp_dist=0.0;
  for(int i(0);i<2;i++) 
    temp_dist+=pow((pos[i]-pos2[i]),2);
  temp_dist=sqrt(temp_dist);

  if(temp_dist<criterion)
    return true;

  return false;
}

void executeCB(const gaze_service::GetHeadAngleGoalConstPtr &goal)
{
      //time_begin_tofind=ros::Time::now().toSec();
      bool success = true;
     // ROS_INFO("%s: succeeded",action_name_.c_str());

     if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        // break;
      }

     //ros::Rate temp_rate(ros_rates);
      if(success)
      {
          feedback_.human_pose = geometry_msgs::PoseStamped();
          result_.is_person=IsHuman;
          std_msgs::Float32 joint_msg;

          desired_target =get_desiredJointAngle(gaze_target_pose);
          feedback_.human_pose.pose.position.x=gaze_target_pose[0];
          feedback_.human_pose.pose.position.y=gaze_target_pose[1];
          
          time_gaze_published=ros::Time::now().toSec();
          joint_msg.data=desired_target;
          Desired_joint_pub.publish(joint_msg);

          as_.setSucceeded(result_);
          as_.publishFeedback(feedback_);
          //ROS_INFO("%s: succeeded",action_name_.c_str());

      }
}


void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    //ROS_INFO("joint states callback");
  Head_Pos[0]=msg->position[9];     //pan
  Head_Pos[1]=msg->position[10];      //tilt
}


double get_desiredJointAngle(std::vector<double>& input_target_pose)
{
    double current_head_angle = Head_Pos[0];
    double desired_angle_target=current_head_angle;
    if(input_target_pose.size()<1)
        return desired_angle_target;
    
    //ROS_INFO("gaze target pose : %.2lf, %.2lf", gaze_target_pose[0],gaze_target_pose[1]);
    //ROS_INFO("global_pose :  %.2lf, %.2lf, %.2lf", global_pose[0],global_pose[1],global_pose[2]);

    geometry_msgs::Vector3Stamped gV, tV;

    gV.vector.x=input_target_pose[0]-global_pose[0];
    gV.vector.y=input_target_pose[1]-global_pose[1];
    gV.vector.z=input_target_pose[2];

    gV.header.stamp-ros::Time();
    gV.header.frame_id="/map";
    listener.transformVector("/base_link",gV,tV);

    //ROS_INFO("before transfomred target pose : %.2lf, %.2lf", gV.vector.x,gV.vector.y);
    //ROS_INFO("transfomred target pose : %.2lf, %.2lf", tV.vector.x,tV.vector.y);

    double target_angle = atan2(tV.vector.y,tV.vector.x);
    //current_head_angle = Head_Pos[0];
    double diff_angle = target_angle-current_head_angle;

    //if(diff)
    ROS_INFO("target angle : %.2lf, head_angle : %.2lf , diff angle %.3lf",target_angle, current_head_angle, diff_angle);

    if(diff_angle>0)
        last_target_direction = 0.8; //left
    else if(diff_angle<0)
        last_target_direction = -0.8;//right

    if(diff_angle>1.0)
        diff_angle=0.7;
    else if(diff_angle<-1.0)
        diff_angle=-0.7;

    desired_angle_target = current_head_angle+diff_angle*0.95;
    //desired_target = current_head_angle+diff_angle*0.7;

    return desired_angle_target; 
}


};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "getheadangle_action");

  HeadAngle_Manager manager(ros::this_node::getName());
  ros::NodeHandle n;
  ros::Subscriber openpose_sub;
  ros::Subscriber global_pos_sub;
  ros::Subscriber joint_state_sub;
  ros::Subscriber edge_leg_sub;
  ros::Subscriber trigger_sub;
  ros::Subscriber target_sub;

  //clicked_point_sub = n.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, &HeadAngle_Manager::ClikedpointCallback,&manager);
  target_sub = n.subscribe<geometry_msgs::PoseStamped>("/target_pose", 10,&HeadAngle_Manager::targetpose_callback, &manager);
  global_pos_sub= n.subscribe<geometry_msgs::PoseStamped>("/global_pose", 10,&HeadAngle_Manager::globalpose_callback, &manager);
  joint_state_sub =n.subscribe<sensor_msgs::JointState>("/hsrb/joint_states", 10, &HeadAngle_Manager::joint_states_callback,&manager);
  trigger_sub=n.subscribe<std_msgs::Int8>("/cmd_trackhuman", 10,&HeadAngle_Manager::trigger_callback,&manager);
  manager.Desired_joint_pub=n.advertise<std_msgs::Float32>("/desired_head_pan", 10, true);
  ros::Rate loop_rate(5);

  double ros_rate = 0.3;
  ros::Rate r(ros_rate);

  ros::spin();

  return 0;
}




