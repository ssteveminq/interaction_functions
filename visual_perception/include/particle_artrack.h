#ifndef PARTICLE_PEOPLE
#define PARTICLE_PEOPLE

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <openpose_ros_wrapper_msgs/Persons3d.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>


#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <string.h>
#include <vector>
#include <math.h>

#define BASE_LINK "/base_link"

#define CAMERA_PIXEL_WIDTH 640
#define CAMERA_PIXEL_HEIGHT 480


class Particle_person
{
public:
    Particle_person();

private:
    void createVisualisation(std::vector<geometry_msgs::Pose> points);
    std::vector<double> cartesianToPolar(geometry_msgs::Point point);
    void openpose3d_callback(const openpose_ros_wrapper_msgs::Persons3d::ConstPtr& msg);
    void ar_pose_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
    void publish_poses(std::vector<geometry_msgs::Pose> points);
    void global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);
    void particle_filter_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void cloud_callback(const sensor_msgs::PointCloud2 &msg);
    void point_3d__to_image();

    std::vector<double> particle_result;
    std::vector<double> face_result;
    std::vector<double> Head_Pos;
    std::vector<double> Head_vel;
    tf::TransformListener listener;
    ros::Publisher  particle_person_pub;
    ros::Publisher  face_person_pub;
    ros::Publisher  point_pub;
    ros::Publisher  people_pose_pub;
    ros::Subscriber people_boxes_sub;
    ros::Subscriber op_people_sub;
    ros::Subscriber global_pos_sub;
    ros::Subscriber joint_states_sub;
    ros::Subscriber pcl_sub;
    ros::Subscriber object_sub;
    ros::Subscriber face_sub;
    ros::Subscriber ar_marker_sub;
    //tf::TransformListener* listener;
    std::string target_frame;
    unsigned long detect_seq;
    unsigned long marker_seq;
    double startup_time;
    std::string startup_time_str;
   
	bool IsRobotMoving;
	bool IsHeadMoving;

	std::vector<double> global_pose;
	std::vector<double> pre_global_pose;
   
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    geometry_msgs::PointStamped particle_output_person;
    geometry_msgs::PointStamped face_output_person;
    //rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
};

#endif // HUMAN_MARKER_H
