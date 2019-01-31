#include "particle_artrack.h"

Particle_person::Particle_person() :
    detect_seq(0),
    marker_seq(0),
    IsHeadMoving(false),
    IsRobotMoving(false),
    cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    ros::NodeHandle n;

    target_frame="map";
    //target_frame="head_rgbd_sensor_rgb_frame";

    // listener = new tf::TransformListener();
    op_people_sub=n.subscribe<openpose_ros_wrapper_msgs::Persons3d>("/openpose/pose_3d", 10, &Particle_person::openpose3d_callback,this);
    object_sub=n.subscribe<std_msgs::Float32MultiArray>("/object_tracker/positions", 10, &Particle_person::particle_filter_callback,this);
	pcl_sub = n.subscribe( "/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 1, &Particle_person::cloud_callback,this);
    ar_marker_sub = n.subscribe( "/ar_pose_marker", 1, &Particle_person::ar_pose_callback,this);

    particle_person_pub=n.advertise<geometry_msgs::PointStamped>("/particle_person", 10, true);
    //face_person_pub=n.advertise<geometry_msgs::PointStamped>("/face_person", 10, true);
    //point_pub=n.advertise<geometry_msgs::PointStamped>("/point_3d_sensoframe", 10, true);
    point_pub=n.advertise<geometry_msgs::PointStamped>("/point_3d", 10, true);
    people_pose_pub=n.advertise<geometry_msgs::PoseArray>("/openpose_pose_array", 10, true);
    global_pos_sub= n.subscribe<geometry_msgs::PoseStamped>("/global_pose", 10,&Particle_person::global_pose_callback, this);
    joint_states_sub= n.subscribe<sensor_msgs::JointState>("/hsrb/joint_states", 10,&Particle_person::joint_states_callback, this);

    //visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_link","/rviz_visual_markers_"));

    global_pose.resize(3,0.0);
    pre_global_pose.resize(3,0.0);
    Head_Pos.resize(2,0.0);
    Head_vel.resize(2,0.0);
    particle_result.resize(2,0.0);
    face_result.resize(2,0.0);
    ros::spin();
}

void Particle_person::point_3d__to_image()
{
    geometry_msgs::PointStamped input_point;
    input_point.point.x=1.0;
    input_point.point.y=-0.09;
    input_point.point.z=0.88;
    input_point.header.frame_id = "map";

    //visualize()
    geometry_msgs::PointStamped output_point;
    listener.transformPoint("head_rgbd_sensor_rgb_frame", input_point, output_point);
    output_point.header.frame_id = "head_rgbd_sensor_rgb_frame";

    double f_x=539.1;
    double f_y=536.464;
    double x_0=315.92;
    double y_0=237.7446;
    
    double u = f_x*output_point.point.x+x_0*output_point.point.z;
    double v = f_x*output_point.point.y+x_0*output_point.point.z;
    double d = output_point.point.z;

    //Camera Intrinsic Matrix
    //K=[535.9208352116473, 0.0, 315.9266712543807, 0.0, 536.4641682047819, 237.7446444724739, 0.0, 0.0, 1.0]
    ROS_INFO("image coordinates - u: %.3lf , v: %.3lf, d: %.3lf",u,v,d);
    point_pub.publish(output_point);

}


void Particle_person::cloud_callback(const sensor_msgs::PointCloud2 &msg)
{
    //ROS_INFO("cloud callback");
    pcl::fromROSMsg(msg, *cloud);
}



void Particle_person::ar_pose_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    ROS_INFO("ar_marker_callback");
    geometry_msgs::PoseStamped input_pose=msg->markers[0].pose;
    geometry_msgs::PointStamped input_point;
    input_point.point.x=input_pose.pose.position.x;
    input_point.point.y=input_pose.pose.position.y;
    input_point.point.z=input_pose.pose.position.z;
    input_point.header.frame_id = msg->markers[0].header.frame_id;

    //visualize()
    geometry_msgs::PointStamped output_point;
    listener.transformPoint("head_rgbd_sensor_rgb_frame", input_point, output_point);
    output_point.header.frame_id = "head_rgbd_sensor_rgb_frame";

    double f_x=539.1;
    double f_y=536.464;
    double x_0=315.92;
    double y_0=237.7446;
    
    double u = f_x*output_point.point.x+x_0*output_point.point.z;
    double v = f_x*output_point.point.y+x_0*output_point.point.z;
    double d = output_point.point.z;

    //Camera Intrinsic Matrix
    //K=[535.9208352116473, 0.0, 315.9266712543807, 0.0, 536.4641682047819, 237.7446444724739, 0.0, 0.0, 1.0]
    ROS_INFO("image coordinates - u: %.3lf , v: %.3lf, d: %.3lf",u,v,d);
    point_pub.publish(output_point);

}


void Particle_person::openpose3d_callback(const openpose_ros_wrapper_msgs::Persons3d::ConstPtr& msg)
{
    //people_pose_array.clear();
    //for(size_t idx(0); i< msg->persons.size();idx++)
        //people_pose_array.push_back(msg->persons[idx].avg_pose);
    
    marker_seq=0;
    int num_of_detected_human=msg->persons.size();
    std::vector<geometry_msgs::Pose> poseVector;

    //ROS_INFO("openpose_callback : people_size : %d", num_of_detected_human);

    if(num_of_detected_human>0)
       poseVector.clear();
    else
    {
      return;
    }

    for(int i(0);i<num_of_detected_human;i++)
    {

      //poseVector.push_back(msg->persons[i].avg_pose.pose);
      float pos_x = msg->persons[i].avg_pose.pose.position.x;
      float pos_y = msg->persons[i].avg_pose.pose.position.y;
      float pos_z = msg->persons[i].avg_pose.pose.position.z;
      //ROS_INFO("pose push_back : people_size : %.3lf , y : %.3f,  z %.3f ", pos_x,pos_y,pos_z);
      geometry_msgs::Vector3Stamped gV, tV;

      gV.vector.x = pos_x;
      gV.vector.y =pos_y;
      gV.vector.z =pos_z;

      //std::cout<<"x :"<<_x<<"_y:"<<_y<<"_z:"<<_z<<std::endl;
      gV.header.stamp = ros::Time();
      gV.header.frame_id = "/head_rgbd_sensor_rgb_frame";
      listener.transformVector("/map", gV, tV);

      geometry_msgs::Pose transformPos;
      transformPos.position.x=tV.vector.x+global_pose[0];
      transformPos.position.y=tV.vector.y+global_pose[1];
      transformPos.position.z=0.5;
      transformPos.orientation.x=0.0;
      transformPos.orientation.y=0.0;
      transformPos.orientation.z=0.0;
      transformPos.orientation.w=1.0;

      poseVector.push_back(transformPos);
    }
  
    //createVisualisation(poseVector);
    publish_poses(poseVector);
    //ROS_INFO(
}

void Particle_person::particle_filter_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    particle_output_person.header.frame_id ="head_rgbd_sensor_rgb_frame";

    //ROS_INFO("partilce callback");
    for(size_t i=0;i<2;i++)
        particle_result[i]=msg->data[i];

    int point_idx = particle_result[0]+particle_result[1]*CAMERA_PIXEL_WIDTH;


    //ROS_INFO("partilce callback--size of pcl: %d ", cloud->points.size());
    int size_pcl=cloud->points.size();
    if(size_pcl>0){
        if ((point_idx<0) || (!pcl::isFinite(cloud->points[point_idx]))){

            //ROS_INFO("partilce callback-----");
            return;
        }
        else{
            //ROS_INFO("partilce callback-----");
            particle_output_person.point.x= cloud->points[point_idx].x;
            particle_output_person.point.y = cloud->points[point_idx].y;
            particle_output_person.point.z = cloud->points[point_idx].z;
        }

        //ROS_INFO("partilce point");
        listener.waitForTransform("map", "head_rgbd_sensor_rgb_frame", ros::Time(0), ros::Duration(3.0));
        //listener.lookupTransform("map", "head_rgbd_sensor_link", ros::Time(0), ros::Duration(3.0));
        particle_output_person.header.frame_id = "head_rgbd_sensor_rgb_frame";
        geometry_msgs::PointStamped point_out;
        listener.transformPoint("map", particle_output_person, point_out);
        particle_output_person.point = point_out.point;
        particle_output_person.header.frame_id = "map";

        particle_person_pub.publish(particle_output_person);
    }
        //ROS_INFO("partilce point");
}

void Particle_person::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{

	Head_Pos[0]=msg->position[9];			//pan
	Head_Pos[1]=msg->position[10];			//tilt
	
	Head_vel[0]=msg->velocity[9];
	Head_vel[1]=msg->velocity[10];


    double head_vel =0.0;
    head_vel = pow(Head_vel[0],2)+pow(Head_vel[1],2);
    head_vel = sqrt(head_vel);

	//ROS_INFO("Head moving : %.3lf",head_vel);

    if(head_vel>0.007)
        IsHeadMoving=true;
    else
        IsHeadMoving=false;
    
    point_3d__to_image();


}

void Particle_person:: publish_poses(std::vector<geometry_msgs::Pose> input_poses){
    
    if(IsHeadMoving || IsRobotMoving )
        return;
    else
    {
        geometry_msgs::PoseArray poses_array_msg;

        poses_array_msg.header.frame_id =target_frame;
        poses_array_msg.header.stamp = ros::Time::now();

        for(int i(0);i<input_poses.size();i++)
            poses_array_msg.poses.push_back(input_poses[i]);

        people_pose_pub.publish(poses_array_msg);
    }
}

void Particle_person::global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    global_pose.resize(3);

   tf::StampedTransform baselinktransform;
   listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10.0));
   listener.lookupTransform("map", "base_link", ros::Time(0), baselinktransform);
   double yaw_tf =   tf::getYaw(baselinktransform.getRotation()); 
	
   //update current global pose
	global_pose[0]=msg->pose.position.x;
	global_pose[1]=msg->pose.position.y;
	global_pose[2]=yaw_tf;

   //check distance with previous pose
    double move_distance =0.0;
    for(size_t idx(0);idx<global_pose.size();idx++)
        move_distance+=pow((pre_global_pose[idx]-global_pose[idx]),2);
    move_distance=sqrt(move_distance);

    //ROS_INFO("robot is moving : %.3lf", move_distance);

    if(move_distance>0.007)
        IsRobotMoving=true;
    else
        IsRobotMoving=false;
    

    //save to previous global pose
    for(size_t idx(0);idx<global_pose.size();idx++)
        pre_global_pose[idx]=global_pose[idx];

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "partilce_filter_person");
    Particle_person* pl = new Particle_person();
    return 0;
}
