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
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <geometry_msgs/PoseStamped.h>

#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <gaze_service/gaze_target.h>

using namespace Eigen;


class closest_human_manager{

public:
  explicit closest_human_manager(){
  
  
   global_pose.resize(3,0.0);
  
  }
  ~closest_human_manager(){}

  ros::Publisher Gaze_point_pub;
  ros::Publisher Gaze_activate_pub;
  ros::Publisher setNavTarget_pub;
  ros::ServiceClient m_client; 

  tf::TransformListener     listener;
  geometry_msgs::PoseArray human_poses;

  std::vector<double> global_pose;

  void Publish_gazetarget(float _x, float _y)
  {

      ROS_INFO("x : %.3lf , y : %.3lf", _x,_y);
    

       //setViewpointTarget(leg_target);
      //setNavTarget_pub.publish(Navmsgs);
      //ROS_INFO("navgation published");

  }
 
void globalpose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){

   global_pose.resize(3);

   tf::StampedTransform baselinktransform;
   listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10.0));
   listener.lookupTransform("map", "base_link", ros::Time(0), baselinktransform);
   double yaw_tf =   tf::getYaw(baselinktransform.getRotation()); 

   global_pose[0]=msg->pose.position.x;
   global_pose[1]=msg->pose.position.y;


}


int FindNearesetIdx(const geometry_msgs::PoseArray pose_array)
{
    //target should be already set
    std::vector<double> Distanceset;
    std::vector<double> temp_pose;

    Distanceset.resize(pose_array.poses.size(),0.0);
    temp_pose.resize(2,0.0);
    
    double minDistance=200.0;
    int    minDistance_Idx=0;

      for(int i(0);i<pose_array.poses.size();i++)
      {

        double pos_x = pose_array.poses[i].position.x;
        double pos_y = pose_array.poses[i].position.y;

        Distanceset[i]=getDistance_from_Vec(global_pose,pos_x,pos_y);
        
        if(minDistance>Distanceset[i])
          {
            minDistance=Distanceset[i];
            minDistance_Idx=i;
          }
      }
  
    return minDistance_Idx;

}

double getDistance_from_Vec(std::vector<double> origin, double _x, double _y)
{
    double temp=0.0;
    if(origin.size()>0)
    {
        temp=(origin[0]-_x)*(origin[0]-_x);
        temp+=(origin[1]-_y)*(origin[1]-_y);
        temp=sqrt(temp);
    }

    return temp;
}


void openpose_Callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{

    human_poses = *msg;

    ROS_INFO("open_pose_callback");

}

void publish_gaze_to_closest_person()
{
    if(human_poses.poses.size()<1)
      return; 


    int closest_human_idx = FindNearesetIdx(human_poses);
    ROS_INFO("closest human idx : %d, input poses size :%d ", closest_human_idx, static_cast<int>(human_poses.poses.size()));

    if(closest_human_idx>human_poses.poses.size())
        return;
    else{
    

        double x_map=human_poses.poses[closest_human_idx].position.x;
        double y_map=human_poses.poses[closest_human_idx].position.y;

        gaze_service::gaze_target gaze_srv;
        gaze_srv.request.x_from_map=x_map;
        gaze_srv.request.y_from_map=y_map;

        printf("Receive point %.3lf , %.3lf \n",x_map,y_map);

        m_client.call(gaze_srv);

    }
}

};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "manager");

  closest_human_manager manager;
    
  ros::NodeHandle n;
  ros::Subscriber openpose_sub;
  ros::Subscriber global_pos_sub;
  
  //clicked_point_sub = n.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, &closest_human_manager::ClikedpointCallback,&manager);
  openpose_sub = n.subscribe<geometry_msgs::PoseArray>("/openpose_pose_array", 10, &closest_human_manager::openpose_Callback,&manager);
  global_pos_sub= n.subscribe<geometry_msgs::PoseStamped>("/global_pose", 10,&closest_human_manager::globalpose_callback, &manager);

  manager.m_client = n.serviceClient<gaze_service::gaze_target>("/gaze_see_target");
  ros::Rate loop_rate(5);

  double ros_rate = 0.25;
  ros::Rate r(ros_rate);

  while (ros::ok())
  {	   
     manager.publish_gaze_to_closest_person();
     ros::spinOnce();
     r.sleep();
  }

  ros::spin();

  return 0;
}




