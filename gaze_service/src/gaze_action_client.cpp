#include "ros/ros.h"
// #include "gaze_action_node.h"
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
#include <std_msgs/Bool.h>
#include <gaze_service/GazingAction.h> 
#include <actionlib/client/simple_action_client.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_gazing_action");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<gaze_service::GazingAction> ac("gazing_action", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  gaze_service::GazingGoal goal;
  goal.x_map = 10;
  goal.x_map = 3;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}





// class gazeAction
// {
// public:
    
//   gazeAction(std::string name): 
//   as_(nh_, name, boost::bind(&gazeAction::executeCB, this,_1), false),
//   action_name_(name)
//   {
//     Gaze_point_pub= nh_.advertise<geometry_msgs::Point>("/gazed_point_fixing_node/target_point", 50, true);
//     Gaze_activate_pub= nh_.advertise<std_msgs::Bool>("/gazed_point_fixing_node/activate", 50, true);

//     //register the goal and feeback callbacks
//     as_.registerPreemptCallback(boost::bind(&gazeAction::preemptCB, this));

//     //subscribe to the data topic of interest
//     // sub_ = nh_.subscribe("/random_number", 1, &gazeAction::analysisCB, this);
//     as_.start();
//   }

//   ~gazeAction(void)
//   {
//   }

//   void preemptCB()
//   {
//     ROS_INFO("%s: Preempted", action_name_.c_str());
//     // set the action state to preempted
//     as_.setPreempted();
//   }

//   void executeCB(const gaze_service::GazingGoalConstPtr &goal)
//   {

//     // ros::Rate r(1);
//      bool success = true;

//      // ROS_INFO("%s: succeeded",action_name_.c_str());

//      if (as_.isPreemptRequested() || !ros::ok())
//       {
//         ROS_INFO("%s: Preempted", action_name_.c_str());
//         // set the action state to preempted
//         as_.setPreempted();
//         success = false;
//         // break;
//       }


//       feedback_.is_possilbe_see=true;
//       // publish the feedback
//       as_.publishFeedback(feedback_);

//       if(success)
//       {
//           result_.success=feedback_.is_possilbe_see;
//           setViewpointTarget(goal->x_map,goal->y_map);
//           as_.setSucceeded(result_);
//           ROS_INFO("%s: succeeded",action_name_.c_str());
//       }


//   }

//   void setViewpointTarget(const float x_map, float y_map)
//   {
//   	geometry_msgs::Point GazePoint_msg;

//   	if(x_map==0.0 && y_map==0.0)
//   	{
//   		GazePoint_msg.x=2.0;
//   		GazePoint_msg.y=0.0;	
//   	}
//   	else
//   	{
//   		GazePoint_msg.x=x_map;
//   		GazePoint_msg.y=y_map;
//   	}

//   	GazePoint_msg.z=1.0;
//   	Gaze_point_pub.publish(GazePoint_msg);

//   	std_msgs::Bool activateGaze_msg;
//   	activateGaze_msg.data=true;

//   	Gaze_activate_pub.publish(activateGaze_msg);
// 	// ros::Duration(0.5).sleep();

//   }


// protected:
    
//   ros::NodeHandle nh_;
//   actionlib::SimpleActionServer<gaze_service::GazingAction> as_;
//   std::string action_name_;
//   int data_count_, goal_;
//   float sum_, sum_sq_;
//   gaze_service::GazingFeedback feedback_;
//   gaze_service::GazingResult result_;
//   ros::Subscriber sub_;
//   ros::Publisher Gaze_point_pub;
//   ros::Publisher Gaze_activate_pub;
// };

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "gazing_action_node");

//   gazeAction gazeaction_node(ros::this_node::getName());
//   ros::spin();

//   return 0;
// }
