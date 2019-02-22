#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
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
#include <gaze_service/FindPersonAction.h>
#include <actionlib/server/simple_action_server.h>

#define TIME_INTERVAL_GAZE 1.5
#define DISTANCE_THRESHOLD 2.0
using namespace std;

class FindPersonManager {
public:

    FindPersonManager(std::string name) :
            as_(nh_, name, boost::bind(&FindPersonManager::executeCB, this, _1), false),
            action_name_(name),
            flag(0),
            flagiter(0),
            time_begin_tofind(0.0),
            IsHuman(false) {
        openpose_sub = nh_.subscribe<geometry_msgs::PoseArray>("/openpose_pose_array", 10,
                                                               &FindPersonManager::openpose_callback, this);
        //while (ros::ok() && openpose_sub.getNumPublishers() == 0) {
            //ROS_WARN_THROTTLE(5, "Waiting for human tracker to come online...");
            //ros::spinOnce();
            //ros::Duration(1).sleep();
        //}
        Gaze_desiredjoint_pub = nh_.advertise<std_msgs::Float32>("/desired_head_pan", 50, true);
        Human_pub= nh_.advertise<std_msgs::Bool>("/Is_Human", 50, true);
        //register the goal and feeback callbacks
        as_.start();
    }

    ~FindPersonManager() = default;

    void openpose_callback(const geometry_msgs::PoseArray::ConstPtr &msg) {
        if (msg->poses.empty()) {
            IsHuman = false;
            feedback_.human_pose.pose = geometry_msgs::Pose();
            return;
        } else {
            IsHuman = false;

            human_poses=*msg;

            std::vector<double> target_area(2,0.0);
            target_area[0]=1.1;
            target_area[1]=-4.0;
            double dist_target=0.0;
            
            feedback_.human_pose.pose = msg->poses.at(0);

            for(size_t idx=0;idx<human_poses.poses.size();idx++)
            {
                //calculate distance between the target
                dist_target+=pow(human_poses.poses[idx].position.x-target_area[0],2);
                dist_target+=pow(human_poses.poses[idx].position.y-target_area[1],2);
                dist_target=sqrt(dist_target);

                if(dist_target<DISTANCE_THRESHOLD)
                {
                    feedback_.human_pose.pose = msg->poses.at(idx);
                    IsHuman=true;
                    //ROS_INFO("human position: x: %.3lf, y: %.3lf, " , msg->poses[idx].position.x, msg->poses[idx].position.y);
                    std_msgs::Bool is_human;
                    is_human.data=true;
                    Human_pub.publish(is_human);
                    return;
                }
                
            }
            
            //gaze_target_pose[0]=human_poses.poses[closest_human_idx].position.x;
            //gaze_target_pose[1]=human_poses.poses[closest_human_idx].position.y;
            //gaze_target_pose[2]=0.5;

            //Pre_IsHuman=true;
        }
    }


    void executeCB(const gaze_service::FindPersonGoalConstPtr &goal) {
        IsHuman = false;
        bool ready_for_next_position = true;
        ros::Time last_goal_timestamp = ros::Time::now();
        time_begin_tofind = ros::Time::now().toSec();

        bool success = true;


        feedback_.human_pose = geometry_msgs::PoseStamped();
        while (true) {
            //as_.publishFeedback(feedback_);
            // Did we see a person?
            if (IsHuman) {
                result_.is_person = true;
                as_.publishFeedback(feedback_);
                ros::spinOnce();
                as_.setSucceeded(result_);
                return;
            }
            // Have we timed out?
            double time_passed = ros::Time::now().toSec() - time_begin_tofind;
            if (time_passed > 10.0) {
                setViewpointTarget(0);
                as_.setAborted(result_);
                return;
            }
            // Have we been preempted?
            if (as_.isPreemptRequested() || !ros::ok()) {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                return;
            }
            if (ros::Time::now() - last_goal_timestamp > ros::Duration(3.0)) {
                ready_for_next_position = true;
                last_goal_timestamp = ros::Time::now();
            }
            // Start the movement
            if (ready_for_next_position) {
                flagiter++;
                int flag_num = flagiter % 4;
                setViewpointTarget(flag_num);
                ready_for_next_position = false;

            }
            ros::spinOnce();
        }
    }

    void setViewpointTarget(int flag) {
        double desiredvalue = 0.0;
        if (flag == 0)
            desiredvalue = -0.5;
        else if (flag == 1)
            desiredvalue = -0.9;
        else if (flag == 2)
            desiredvalue = -1.0;
        else if (flag == 3)
            desiredvalue = -1.0;
        else
            desiredvalue = 0.0;

        std_msgs::Float32 head_pan_msg;
        head_pan_msg.data = desiredvalue;
        Gaze_desiredjoint_pub.publish(head_pan_msg);

    }

protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<gaze_service::FindPersonAction> as_;
    std::string action_name_;
    int data_count_, goal_;
    float sum_, sum_sq_;
    gaze_service::FindPersonFeedback feedback_;
    gaze_service::FindPersonResult result_;
    ros::Subscriber sub_;
    ros::Subscriber openpose_sub;
    ros::Publisher Gaze_desiredjoint_pub;
    ros::Publisher Human_pub;
    int flag;
    int flagiter;
    geometry_msgs::PoseArray human_poses;
    bool IsHuman;
    double time_begin_tofind;


};

int main(int argc, char **argv) {
    ros::init(argc, argv, "findperson_action");
    FindPersonManager find_person_node(ros::this_node::getName());
    ros::spin();

    return 0;
}
