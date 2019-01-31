#include "gaze_service_node.h"


gaze_srv::gaze_srv(){
	}
gaze_srv::~gaze_srv(){}


void gaze_srv::Publish_nav_target(float _x, float _y, float _theta)
{
		ROS_INFO("x : %.3lf , y : %.3lf", _x,_y);
	
		move_base_msgs::MoveBaseActionGoal Navmsgs;
		Navmsgs.header.stamp =  ros::Time::now();
		Navmsgs.goal.target_pose.header.frame_id = "map";

		 Navmsgs.goal.target_pose.pose.position.x=_x;
		 Navmsgs.goal.target_pose.pose.position.y=_y;
		 Navmsgs.goal.target_pose.pose.position.z=0.0;

		 Navmsgs.goal.target_pose.pose.orientation.x=0.0;
		 Navmsgs.goal.target_pose.pose.orientation.y=0.0;
		 Navmsgs.goal.target_pose.pose.orientation.z=0.0;
		 Navmsgs.goal.target_pose.pose.orientation.w=1.0;

		 // setViewpointTarget(leg_target);
		 setNavTarget_pub.publish(Navmsgs);
		 ROS_INFO("navgation published");
}


bool gaze_srv::seeTarget(gaze_service::gaze_target::Request &req, gaze_service::gaze_target::Response &res)
{

	if(req.x_from_map==NULL)
	{
		//setViewpointTarget(req.x_from_map,req.y_from_map);
		res.is_possible_see=false;
	}
	else
	{
		setViewpointTarget(req.x_from_map,req.y_from_map);
		res.is_possible_see=true;
	}
}



void gaze_srv::setViewpointTarget(const float x_map, float y_map)
{
	geometry_msgs::Point GazePoint_msg;

	if(x_map==0.0 && y_map==0.0)
	{
		GazePoint_msg.x=2.0;
		GazePoint_msg.y=0.0;	
	}
	else
	{
		GazePoint_msg.x=x_map;
		GazePoint_msg.y=y_map;
        GazePoint_msg.z=1.0;
        Gaze_point_pub.publish(GazePoint_msg);

        std_msgs::Bool activateGaze_msg;
        activateGaze_msg.data=true;
        Gaze_activate_pub.publish(activateGaze_msg);

	}

	//GazePoint_msg.z=1.0;
	//Gaze_point_pub.publish(GazePoint_msg);

	//std_msgs::Bool activateGaze_msg;
	//activateGaze_msg.data=true;

	//Gaze_activate_pub.publish(activateGaze_msg);
     ros::Duration(1.0).sleep();
}










