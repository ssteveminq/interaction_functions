#include "octomap_checker.h"

Octomap_manager::Octomap_manager() :
    detect_seq(0),
    marker_seq(0),
    IsHeadMoving(false),
    IsRobotMoving(false),
    octree(NULL),
    cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    ros::NodeHandle n;

    target_frame="map";
    //target_frame="head_rgbd_sensor_rgb_frame";

    // listener = new tf::TransformListener();
    //op_people_sub=n.subscribe<openpose_ros_wrapper_msgs::Persons3d>("/openpose/pose_3d", 10, &Octomap_manager::openpose3d_callback,this);
    octomap_sub=n.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, &Octomap_manager::octomap_callback,this);

    //face_person_pub=n.advertise<geometry_msgs::PointStamped>("/face_person", 10, true);
    //point_pub=n.advertise<geometry_msgs::PointStamped>("/point_3d_sensoframe", 10, true);
    point_pub=n.advertise<geometry_msgs::PointStamped>("/point_3d", 10, true);
    imagepoint_pub=n.advertise<geometry_msgs::PointStamped>("/imagepoint", 10, true);
    people_pose_pub=n.advertise<geometry_msgs::PoseArray>("/openpose_pose_array", 10, true);
    global_pos_sub= n.subscribe<geometry_msgs::PoseStamped>("/global_pose", 10,&Octomap_manager::global_pose_callback, this);
    joint_states_sub= n.subscribe<sensor_msgs::JointState>("/hsrb/joint_states", 10,&Octomap_manager::joint_states_callback, this);

    //visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_link","/rviz_visual_markers_"));

    global_pose.resize(3,0.0);
    pre_global_pose.resize(3,0.0);
    Head_Pos.resize(2,0.0);
    Head_vel.resize(2,0.0);
    particle_result.resize(2,0.0);
    face_result.resize(2,0.0);
    ros::spin();
}
Octomap_manager::~Octomap_manager()
{

    delete octree;
    octree=NULL;

}

void Octomap_manager::point_3d__to_image()
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
    //ROS_INFO("image coordinates - u: %.3lf , v: %.3lf, d: %.3lf",u,v,d);
    point_pub.publish(output_point);

}


void Octomap_manager::cloud_callback(const sensor_msgs::PointCloud2 &msg)
{
    //ROS_INFO("cloud callback");
    pcl::fromROSMsg(msg, *cloud);
}



void Octomap_manager::ar_pose_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    //ROS_INFO("ar_marker_callback");
    if(msg->markers.size()<1)
        return;
     
    geometry_msgs::PoseStamped input_pose=msg->markers[0].pose;
    geometry_msgs::PointStamped input_point;
    input_point.point.x=input_pose.pose.position.x;
    input_point.point.y=input_pose.pose.position.y;
    input_point.point.z=input_pose.pose.position.z;
    input_point.header.frame_id = msg->markers[0].header.frame_id;

    //visualize()
    geometry_msgs::PointStamped output_point;
    listener.waitForTransform("map", "head_rgbd_sensor_rgb_frame", ros::Time(0), ros::Duration(10.0));
    listener.transformPoint("head_rgbd_sensor_rgb_frame", input_point, output_point);
    output_point.header.frame_id = "head_rgbd_sensor_rgb_frame";

    double f_x=539.1;
    double f_y=536.464;
    double x_0=315.92;
    double y_0=237.7446;
    
    ROS_INFO("point x: %.3lf, y: %.3lf, z: %.3lf",output_point.point.x,output_point.point.y, output_point.point.z);
    if(output_point.point.x==NULL)
        return;

    //K=[535.9208352116473, 0.0, 315.9266712543807, 0.0, 536.4641682047819, 237.7446444724739, 0.0, 0.0, 1.0]
    
    point_pub.publish(output_point);

    double u = f_x*output_point.point.x+x_0*output_point.point.z;
    double v = f_x*output_point.point.y+x_0*output_point.point.z;
    double d = output_point.point.z;

    ROS_INFO("image coordinates - u: %.3lf , v: %.3lf, d: %.3lf",u,v,d);
    geometry_msgs::PointStamped image_point;
    image_point.point.x=u;
    image_point.point.y=v;
    image_point.point.z=d;
    imagepoint_pub.publish(image_point);

}


void Octomap_manager::openpose3d_callback(const openpose_ros_wrapper_msgs::Persons3d::ConstPtr& msg)
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

void Octomap_manager::octomap_callback(const octomap_msgs::Octomap::ConstPtr& msg)
{
      ROS_INFO("octomap_callback");

      AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
      //OcTree* octree = NULL;

      if (tree){
        octree = dynamic_cast<OcTree*>(tree);
        ROS_INFO("get tree size: %zu resolution:%f, ",octree->size(),octree->getResolution());
        point3d query_ (1.2, 1.0, 1.0);
        OcTreeNode* result = octree->search(query_);
        if(result !=NULL)
            std::cout<<"occ probability at "<<query_<<" : "<<result->getOccupancy()<<std::endl;
        else
            std::cout<<"occ probability at "<<query_<<" : "<<"unkown"<<std::endl;

        OcTree::leaf_iterator it=octree->begin_leafs();
        OcTree::leaf_iterator end=octree->end_leafs();

        for(it;it!=end;++it)
        {

            //std::cout<<"Node center:"<<it.getCoordinate()<<std::endl;
        }


        //ROS_INFO("x:%.2lf,y:%.2lf, z:%.2lf", )
        //leaf_iterator

      } else {
        ROS_ERROR("Error creating octree from received message");
        //if (resp.map.id == "ColorOcTree")
          //ROS_WARN("You requested a binary map for a ColorOcTree - this is currently not supported. Please add -f to request a full map");
      }



}

void Octomap_manager::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
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
    
    //point_3d__to_image();


}

void Octomap_manager:: publish_poses(std::vector<geometry_msgs::Pose> input_poses){
    
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

void Octomap_manager::global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
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
    Octomap_manager* pl = new Octomap_manager();
    return 0;
}
