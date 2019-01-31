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

#include <ar_track_alvar_msgs/AlvarMarker.h> 
#include <ar_track_alvar_msgs/AlvarMarkers.h> 

using namespace Eigen;

using namespace std;
//using namespace laser_processor;
//using namespace ros;
//using namespace tf;
//using namespace estimation;
//using namespace BFL;
//using namespace MatrixWrapper;

static const string pose_scan_topic = "openpose_pose_array";

static const double no_observation_timeout_s = 0.5;
static const double max_second_leg_age_s     = 2.0;
static const double max_track_jump_m         = 1.0; //1.0;
static const double max_meas_jump_m          = 0.75; //0.75; // 1.0
//static const string fixed_frame              = "odom_combined";
static const string fixed_frame              = "/map";
static const string GLOBAL_FRAME              = "map";
static double cov_meas_legs_m          = 0.025;
static double cov_meas_people_m        = 0.025;

static const double det_dist__for_pause      = 0.5; // the distance to the person when the robot decides to pause
static const double det_dist__for_resume      = 2.5; // the distance to the person when the robot decides to resume
static double kal_p =4, kal_q = 0.002, kal_r = 10;
static bool use_filter = true;
static string detector_="hsrb/base_scan";

/*
class SavedFeature
{
public:
	static int nextid;
	TransformListener& tfl_;

	BFL::StatePosVel sys_sigma_;
	TrackerKalman filter_;

	string id_;
	string object_id;
	ros::Time time_;
	ros::Time meas_time_;

	double reliability, p, probability;

	Stamped<Point> position_;
	Stamped<Vector3> velocity_;
	SavedFeature* other;
	float dist_to_person_;


	// one leg tracker
	SavedFeature(Stamped<Point> loc, TransformListener& tfl)
	: tfl_(tfl),
	  sys_sigma_(Vector3(0.05, 0.05, 0.05), Vector3(1.0, 1.0, 1.0)),
	  filter_("tracker",sys_sigma_),
	  reliability(-1.), p(4)
	{
		char id[100];
		//snprintf(id,100,"legtrack %d", nextid++);
		id_ = std::string(id);

		object_id = "";
		time_ = loc.stamp_;
		meas_time_ = loc.stamp_;
		other = NULL;

		try {
			tfl_.transformPoint(fixed_frame, loc, loc);
		} catch(...) {
			ROS_WARN("TF exception spot 6.");
		}
		StampedTransform pose( Pose(Quaternion(0.0, 0.0, 0.0, 1.0), loc), loc.stamp_, id_, loc.frame_id_);
		tfl_.setTransform(pose);

		StatePosVel prior_sigma(Vector3(0.1,0.1,0.1), Vector3(0.0000001, 0.0000001, 0.0000001));
		filter_.initialize(loc, prior_sigma, time_.toSec());
		StatePosVel est;
		filter_.getEstimate(est);
		updatePosition();

	}

	void propagate(ros::Time time)
	{
		time_ = time;
		filter_.updatePrediction(time.toSec());
		updatePosition();
	}

	void update(Stamped<Point> loc, double probability)
	{
		StampedTransform pose( Pose(Quaternion(0.0, 0.0, 0.0, 1.0), loc), loc.stamp_, id_, loc.frame_id_);
		tfl_.setTransform(pose);

		meas_time_ = loc.stamp_;
		time_ = meas_time_;

		SymmetricMatrix cov(3);
		cov = 0.0;
		cov(1,1) = cov_meas_legs_m;
		cov(2,2) = cov_meas_legs_m;
		cov(3,3) = cov_meas_legs_m;

		filter_.updateCorrection(loc, cov);
		updatePosition();

		if(reliability<0 || !use_filter){
			reliability = probability;
			p = kal_p;
		}
		else{
			p += kal_q;
			double k = p / (p+kal_r);
			reliability += k * (probability - reliability);
			p *= (1 - k);
		}
	}

	double getLifetime()
	{
		return filter_.getLifetime();
	}

	double getReliability()
	{
		return reliability;
	}

private:
	void updatePosition()
	{
		StatePosVel est;
		filter_.getEstimate(est);

		position_[0] = est.pos_[0];
		position_[1] = est.pos_[1];
		position_[2] = est.pos_[2];

		velocity_[0] = est.vel_[0];
		velocity_[1] = est.vel_[1];
		velocity_[2] = est.vel_[2];

		position_.stamp_ = time_;
		position_.frame_id_ = fixed_frame;
		velocity_.stamp_ = time_;
		velocity_.frame_id_ = fixed_frame;

		double nreliability = fmin(1.0, fmax(0.1, est.vel_.length() / 0.5));
	}

};

int SavedFeature::nextid = 0;

*/


class ar_tracker{

public:
  explicit ar_tracker(){}
  ~ar_tracker(){}

  ros::Publisher Gaze_activate_pub;
  ros::ServiceClient m_client; 

  tf::TransformListener     listener;

  void Publish_gazetarget(float _x, float _y)
  {

      ROS_INFO("x : %.3lf , y : %.3lf", _x,_y);
    
  }

  void armarker_Callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& markers_msg)
    {
        //printf("armarker_callback");
        //std::cout<<"armarker_callback";
        ar_track_alvar_msgs::AlvarMarkers data=*markers_msg;
        geometry_msgs::PoseStamped pose;
        int num_objects = data.markers.size();

        if(num_objects>0)
            ROS_INFO("marker id: %d, marker frame_id : %s ", static_cast<int>(data.markers[0].id),
                                          data.markers[0].pose.header.frame_id.c_str());
    
    }


};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "ar_manager");

  ar_tracker manager;
    
  ros::NodeHandle n;
  ros::Subscriber armarkers_sub;
  
  //clicked_point_sub = n.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, &ar_tracker::ClikedpointCallback,&manager);
  armarkers_sub=n.subscribe<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker",10, &ar_tracker::armarker_Callback, &manager);
  //manager.m_client = n.serviceClient<gaze_service::gaze_target>("/gaze_see_target");
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




