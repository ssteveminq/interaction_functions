#include <string>
#include <boost/thread/mutex.hpp>

// ros stuff
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include <tf/transform_datatypes.h>
#include <actionlib/server/simple_action_server.h>
// messages
#include <std_msgs/Int8.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud.h>
#include <message_filters/time_sequencer.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/OccupancyGrid.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <visual_perception/SearchAction.h>
#include <visual_perception/SearchResult.h>
#include <visual_perception/SearchFeedback.h>
//#include <visualization_msgs/Marker.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <frontier_exploration/Frontier.h>
#include <boost/foreach.hpp>

// log files
#include <fstream>
#include <vector>
#include <queue>

#define FOVW 36       //field of view width
#define MATH_PI 3.14159265359
#define Target_Dist_person 1.0
#define LASER_Dist_person  2.5

#define P_S_given_H 0.8
#define P_S_given_Hc 0.5
#define P_Sc_given_H 0.01
#define P_Sc_given_Hc 0.99

#define HUMANS_DETECTED 1
#define HUMAN_OCCUPIED 1
#define NO_HUMANS_DETECTED 0
#define PROB_THRESH 0.1


using namespace std;

typedef std::pair<double,int> mypair;
bool comparator( const mypair& l, const mypair& r)
{ return l.first < r.first; } 


class belief_manager
{
public:
  /// constructor
  belief_manager(std::string name);
  /// destructor
  virtual ~belief_manager();

  /// callback for messages
  void publish_cameraregion();
  void publish_target_belief();
  void publish_searchmap();
  void publish_human_candidates();
  void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);
  void global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void human_poses_callback(const geometry_msgs::PoseArray::ConstPtr& message);
  void target_poses_callback(const geometry_msgs::PoseArray::ConstPtr& message);
  void scaled_static_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void scaled_dynamic_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void dyn_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  
  bool Comparetwopoistions(std::vector<double> pos,std::vector<double> pos2,double criterion);
  bool check_staticObs(float x_pos,float y_pos);
  bool getlinevalue(int line_type,double input_x, double input_y);
  double getDistance_from_Vec(std::vector<double> origin, double _x, double _y);
  bool check_cameraregion(float x_pos,float y_pos);

  int  globalcoord_To_SScaled_map_index(float x_pos,float y_pos);
  int  CoordinateTransform_Global2_staticMap(float global_x, float global_y);
  int  CoordinateTransform_Global2_beliefMap(double global_x, double global_y);
  void Mapidx2GlobalCoord(int map_idx, std::vector<double>& global_coords);
    
  void update_human_occ_belief(int update_type);
  void getCameraregion();
  void spin();

  void executeCB(const visual_perception::SearchGoalConstPtr &goal);

  //costmap
  void Extract_frontier();

  frontier_exploration::Frontier buildNewFrontier(unsigned int initial_cell, unsigned int reference, std::vector<bool>& frontier_flag);
  bool isNewFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag);
  double frontierCost(const frontier_exploration::Frontier& frontier);
  bool nearestCell(unsigned int &result, unsigned int start, unsigned char val);
  std::vector<unsigned int> nhood8(unsigned int idx);
  std::vector<unsigned int> nhood4(unsigned int idx);
  std::vector<frontier_exploration::Frontier> searchFrom(geometry_msgs::Point position);
  unsigned char* getCharMap() const; 
  void visualizeFrontiers(const std::vector<frontier_exploration::Frontier>& frontiers);
  void getNextFrontiers(const std::vector<frontier_exploration::Frontier>& frontiers);
  void reset_search_map();

private:

  ros::NodeHandle nh_;
  std::string action_name_;
  actionlib::SimpleActionServer<visual_perception::SearchAction> as_;
  visual_perception::SearchFeedback feedback_;
  visual_perception::SearchResult result_;

  ros::Publisher camera_visible_region_pub;
  ros::Publisher human_target_pub;
  ros::Publisher belief_pub;
  ros::Publisher searchmap_pub;
  ros::Publisher human_candidates_pub;
  ros::Publisher frontier_marker_pub;
  ros::Publisher nextfrontier_pose_pub;
  ros::Publisher frontier_cloud_pub;

  ros::Subscriber people_yolo_sub;
  ros::Subscriber people_poses_sub;
  ros::Subscriber globalpose_sub;
  ros::Subscriber Scaled_static_map_sub;
  ros::Subscriber joint_state_sub;
  ros::Subscriber target_poses_sub;

  unsigned int min_frontier_size_;
  unsigned int costmap_size_x;
  unsigned int costmap_size_y;
  unsigned char* costmap_;
  
  // tf listener
  tf::TransformListener     robot_state_;
  tf::TransformListener     listener;

  std::string fixed_frame_;
  boost::mutex filter_mutex_;

  std::vector<int> visiblie_idx_set;
  std::vector<int> human_occupied_idx;

  std::vector<double> global_pose;
  std::vector< std::vector< double > > cur_target;
  std::vector<double> Head_Pos; 

  int num_of_detected_human;
  int num_of_detected_target;
  nav_msgs::OccupancyGrid Scaled_map;
  nav_msgs::OccupancyGrid camera_visible_region;
  nav_msgs::OccupancyGrid dynamic_belief_map;
  nav_msgs::OccupancyGrid Human_Belief_Scan_map;
  nav_msgs::OccupancyGrid Human_Belief_type_map;
  nav_msgs::OccupancyGrid Target_Search_map;
  geometry_msgs::PoseArray human_candidates_poses;
  
  std::vector<double> m_dyn_occupancy;
  std::vector<double> m_prob_occupancy;
	
  
  std::vector<int> index_of_target_occ_cells_updated_recently;
  std::map<int, float> map_index_of_target_cells_to_prob;
  

  //costmap
  double potential_scale_;
  double gain_scale_;
  double blacklist_radius_;
  double map_res;
  int last_markers_count_;
  bool isTargetDetected;
  bool isActionActive;

}; // class


