#include "ros/ros.h"
#include "gaze_service_node.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <sstream>
#include <signal.h>
#include <boost/thread/thread.hpp>
#include <gaze_service/gaze_target.h>
using namespace Eigen;

bool g_caught_sigint=false;

void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "gaze_service_test");

  gaze_srv gaze_srvice_manager;
 
  ros::NodeHandle n;
  // ros::Subscriber global_pos_sub;


  gaze_srvice_manager.setNavTarget_pub=n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/move/goal",50,true);
  gaze_srvice_manager.Gaze_point_pub= n.advertise<geometry_msgs::Point>("/gazed_point_fixing_node/target_point", 50, true);
  gaze_srvice_manager.Gaze_activate_pub= n.advertise<std_msgs::Bool>("/gazed_point_fixing_node/activate", 50, true);
  
  ros::Rate loop_rate(30);

  ros::ServiceServer service = n.advertiseService("/gaze_see_target",  &gaze_srv::seeTarget,&gaze_srvice_manager);
  
  signal(SIGINT, sig_handler);
  double ros_rate = 0.5;
  ros::Rate r(ros_rate);

  while (ros::ok())
  {
	   
     ros::spinOnce();
     r.sleep();
  }

  ros::spin();

  return 0;
}




