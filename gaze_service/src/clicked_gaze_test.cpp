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


class click_gaze_manager{

public:
  explicit click_gaze_manager(){}
  ~click_gaze_manager(){}

  ros::Publisher Gaze_point_pub;
  ros::Publisher Gaze_activate_pub;
  ros::Publisher setNavTarget_pub;
  ros::ServiceClient m_client; 

  tf::TransformListener     listener;

  void Publish_gazetarget(float _x, float _y)
  {

      ROS_INFO("x : %.3lf , y : %.3lf", _x,_y);
    
       //setViewpointTarget(leg_target);
      //setNavTarget_pub.publish(Navmsgs);
      //ROS_INFO("navgation published");

  }
 
  // void setViewpointTarget(const std::vector<double> pos);
  void ClikedpointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
  {
      double x_map=msg->point.x;
      double y_map=msg->point.y;
      double theta_map=0.0;      

      gaze_service::gaze_target gaze_srv;
      gaze_srv.request.x_from_map=x_map;
      gaze_srv.request.y_from_map=y_map;
      
       printf("Receive point %.3lf , %.3lf \n",x_map,y_map);
      
      m_client.call(gaze_srv);
       //printf("Receive point %.3lf , %.3lf \n",x_map,y_map);

  }

};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "manager");

  click_gaze_manager manager;
    
  ros::NodeHandle n;
  ros::Subscriber clicked_point_sub;
  
  clicked_point_sub = n.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, &click_gaze_manager::ClikedpointCallback,&manager);
  //manager.setNavTarget_pub=n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/move/goal",50,true);

  manager.m_client = n.serviceClient<gaze_service::gaze_target>("/gaze_see_target");
  ros::Rate loop_rate(50);

  double ros_rate = 3.0;
  ros::Rate r(ros_rate);

  while (ros::ok())
  {	   
     ros::spinOnce();
     r.sleep();
  }

  ros::spin();

  return 0;
}




