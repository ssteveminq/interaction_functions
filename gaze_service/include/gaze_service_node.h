#include <vector>
#include <fstream>
#include <string>
#include <map>
#include <stdint.h>
#include "ros/ros.h"
#include <ros/package.h>
#include <Eigen/Dense>
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PoseArray.h"
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <gaze_service/gaze_target.h>


class gaze_srv{

public:
	gaze_srv();
	~gaze_srv();

	//
	ros::Publisher Gaze_point_pub;
	ros::Publisher Gaze_activate_pub;
	ros::Publisher setNavTarget_pub;
	
	//
	tf::TransformListener 	  listener;

	void Publish_nav_target(float _x, float _y, float _theta);
	// void setViewpointTarget(const std::vector<double> pos);
	bool seeTarget(gaze_service::gaze_target::Request &req, gaze_service::gaze_target::Response &res);	
	void setViewpointTarget(const float x_map, float y_map);

	

};
