#include "human_belief.h"

using namespace std;
using namespace tf;
using namespace message_filters;

static const double       sequencer_delay            = 0.5; //TODO: this is probably too big, it was 0.8
static const unsigned int sequencer_internal_buffer  = 100;
static const unsigned int sequencer_subscribe_buffer = 10;
static const unsigned int num_particles_tracker      = 1000;
static const double       tracker_init_dist          = 4.0;

static const unsigned char  FREE_SPACE = 0;
static const unsigned char  INSCRIBED_INFLATED_OBSTACLE = 253;
static const unsigned char  LETHAL_OBSTACLE = 254;
static const unsigned char  NO_INFORMATION = 255;

bool IsNotInitilized = true;


// constructor
belief_manager::belief_manager(std::string name)
  :as_(nh_, name, boost::bind(&belief_manager::executeCB, this,_1), false),
   action_name_(name),costmap_(NULL),min_frontier_size_(1),potential_scale_(1e-3), gain_scale_(1.0),blacklist_radius_(0.2),map_res(0.5),
   robot_state_(), isTargetDetected(false), isActionActive(false)
{
  // initialize
   
  global_pose.resize(3,0.0);
  Head_Pos.resize(2,0.0);
  double offset_origin = -7.5;

  //camera region
  camera_visible_region.info.width=30;
  camera_visible_region.info.height= 30;
  camera_visible_region.info.resolution=0.5;
  camera_visible_region.info.origin.position.x=offset_origin;
  camera_visible_region.info.origin.position.y=offset_origin;
  camera_visible_region.data.resize(camera_visible_region.info.width*camera_visible_region.info.height,0.0);

  //Initialize human_belief_map
  Human_Belief_Scan_map.info.width=30;
  Human_Belief_Scan_map.info.height= 30;
  Human_Belief_Scan_map.info.resolution=0.5;
  Human_Belief_Scan_map.info.origin.position.x=offset_origin;
  Human_Belief_Scan_map.info.origin.position.y=offset_origin;
  int belief_size=Human_Belief_Scan_map.info.width*Human_Belief_Scan_map.info.height;
  Human_Belief_Scan_map.data.resize(Human_Belief_Scan_map.info.width*Human_Belief_Scan_map.info.height);
  // int belief_map_size=static_belief_map.info.width*static_belief_map.info.height;
  for(int k(0);k<belief_size;k++)
      Human_Belief_Scan_map.data[k]=0.01;

 //Initialize human_belief_type_map
  Human_Belief_type_map.info.width=30;
  Human_Belief_type_map.info.height= 30;
  Human_Belief_type_map.info.resolution=0.5;
  Human_Belief_type_map.info.origin.position.x=offset_origin;
  Human_Belief_type_map.info.origin.position.y=offset_origin;
  Human_Belief_type_map.data.resize(Human_Belief_type_map.info.width*Human_Belief_type_map.info.height);
  for(int k(0);k<belief_size;k++)
      Human_Belief_type_map.data[k]=0.0;

  Target_Search_map.info.width=30;
  Target_Search_map.info.height= 30;
  Target_Search_map.info.resolution=0.5;
  Target_Search_map.info.origin.position.x=offset_origin;
  Target_Search_map.info.origin.position.y=offset_origin;
  Target_Search_map.data.resize(Target_Search_map.info.width*Target_Search_map.info.height);
  unsigned int search_size=Target_Search_map.info.width*Target_Search_map.info.height;
  for(int k(0);k<search_size;k++)
      Target_Search_map.data[k]=-1.0;


  //Initialize costmap variable
  last_markers_count_=0;
  costmap_size_x=Target_Search_map.info.width;
  costmap_size_y=Target_Search_map.info.height;
  map_res = Target_Search_map.info.resolution;
  costmap_ = new unsigned char[search_size];
  memset(costmap_,NO_INFORMATION, search_size * sizeof(unsigned char));

  camera_visible_region_pub=nh_.advertise<nav_msgs::OccupancyGrid>("/camera_region_map", 10, true);
  human_candidates_pub=nh_.advertise<geometry_msgs::PoseArray>("/human_belief_pose_array", 10, true);
  belief_pub=nh_.advertise<nav_msgs::OccupancyGrid>("/human_belief_map", 10, true);
  searchmap_pub=nh_.advertise<nav_msgs::OccupancyGrid>("/target_search_map", 10, true);
  frontier_marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("frontier_list_search", 5);
  frontier_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("frontiers",5);
  nextfrontier_pose_pub= nh_.advertise<geometry_msgs::PoseStamped>("next_frontier",5);

  joint_state_sub =nh_.subscribe<sensor_msgs::JointState>("/hsrb/joint_states", 10, &belief_manager::joint_states_callback,this);
  target_poses_sub =nh_.subscribe<geometry_msgs::PoseArray>("/bottle_poses", 10, &belief_manager::target_poses_callback,this);
  globalpose_sub=nh_.subscribe<geometry_msgs::PoseStamped>("/global_pose",10,&belief_manager::global_pose_callback,this);

  as_.start();
}

// destructor
belief_manager::~belief_manager()
{
    delete []costmap_;
    costmap_ = NULL;
  // delete sequencer
  // delete all trackers
};

void belief_manager::reset_search_map()
{

    if(costmap_ !=NULL)
        delete []costmap_;

  isTargetDetected=false;
  double offset_origin = -7.5;
  Target_Search_map.info.width=30;
  Target_Search_map.info.height= 30;
  Target_Search_map.info.resolution=0.5;
  Target_Search_map.info.origin.position.x=offset_origin;
  Target_Search_map.info.origin.position.y=offset_origin;
  Target_Search_map.data.resize(Target_Search_map.info.width*Target_Search_map.info.height);
  unsigned int search_size=Target_Search_map.info.width*Target_Search_map.info.height;
  for(int k(0);k<search_size;k++)
      Target_Search_map.data[k]=-1.0;

  //Initialize costmap variable
  last_markers_count_=0;
  costmap_size_x=Target_Search_map.info.width;
  costmap_size_y=Target_Search_map.info.height;
  map_res = Target_Search_map.info.resolution;
  costmap_ = new unsigned char[search_size];
  memset(costmap_,NO_INFORMATION, search_size * sizeof(unsigned char));
  ROS_INFO("search_costmap_reset");

}


void belief_manager::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    //ROS_INFO("joint states callback");
  Head_Pos[0]=msg->position[9];     //pan
  Head_Pos[1]=msg->position[10];      //tilt
}


void belief_manager::publish_cameraregion()
{

   getCameraregion();
   camera_visible_region.header.stamp =  ros::Time::now();
   camera_visible_region.header.frame_id = "map"; 
   camera_visible_region_pub.publish(camera_visible_region);

}

void belief_manager::Mapidx2GlobalCoord(int map_idx, std::vector<double>& global_coords)
{
    global_coords.resize(2,0.0);

    std::vector<int> cell_xy(2,0.0);

    int res = map_idx / Human_Belief_type_map.info.width;
    int div = map_idx % Human_Belief_type_map.info.width;
    //ROS_INFO("res: %d, div : %d", res, div);

    cell_xy[0]=div;
    cell_xy[1]=res;

    global_coords[0] =  Human_Belief_type_map.info.origin.position.x+cell_xy[0]*Human_Belief_type_map.info.resolution;
    global_coords[1] =  Human_Belief_type_map.info.origin.position.y+cell_xy[1]*Human_Belief_type_map.info.resolution;
    //ROS_INFO("global coord x: %.lf, div : %.lf", global_coords[0],global_coords[1]);

}

void belief_manager::publish_searchmap()
{
   Target_Search_map.header.stamp =  ros::Time::now();
   Target_Search_map.header.frame_id = "map"; 
   searchmap_pub.publish(Target_Search_map);
}


void belief_manager::publish_target_belief()
{
    typedef std::map<int, float>::iterator map_iter;
    for(map_iter iterator = map_index_of_target_cells_to_prob.begin(); iterator !=  map_index_of_target_cells_to_prob.end(); iterator++) {

        int human_map_idx = iterator->first;
        Human_Belief_Scan_map.data[human_map_idx]=(iterator->second) * 100.0;
    }
    

   Human_Belief_Scan_map.header.stamp =  ros::Time::now();
   Human_Belief_Scan_map.header.frame_id = "map"; 
   belief_pub.publish(Human_Belief_Scan_map);
   
   /*
   human_candidates_poses.poses.clear();

   for(size_t idx=0;idx<Human_Belief_Scan_map.data.size();idx++)
   {
         if(Human_Belief_Scan_map.data[idx]>10)
         {
             std::vector<double> temp_pose(2,0.0);
             Mapidx2GlobalCoord(idx,temp_pose);

             geometry_msgs::Pose human_pose;
             human_pose.position.x=temp_pose[0];
             human_pose.position.y=temp_pose[1];
             human_pose.position.z=0.5;
             human_pose.orientation.x=0.0;
             human_pose.orientation.y=0.0;
             human_pose.orientation.z=0.0;
             human_pose.orientation.w=1.0;
         
             human_candidates_poses.poses.push_back(human_pose);

         }
   }

   std::vector<std::pair<double,int> > human_pairs;
   std::vector<double> human_candidates_distances;

   size_t candidate_size = human_candidates_poses.poses.size();
   human_candidates_distances.resize(candidate_size,0.0);
   for(size_t i(0);i<candidate_size;i++)
   {
       human_candidates_distances[i]=getDistance_from_Vec(global_pose,
                                                          human_candidates_poses.poses[i].position.x,
                                                          human_candidates_poses.poses[i].position.y);
   
       std::pair<double, int> temp_pair(human_candidates_distances[i],i);
       human_pairs.push_back(temp_pair);
       //human_pairs.push_back(std::pair<double, int>(human_candidates_distances[i],i));
   }

   //print functions
   //ROS_INFO("---------------------before sorting------------------------------");
   for(size_t j(0);j<human_pairs.size();j++)
       ROS_INFO("human pair index: distance = idx: %d, dist:  %.3lf", human_pairs[j].first, human_pairs[j].second);
   //ROS_INFO("------------------------------------------after sorting -------------");
   //This sorting function saves the distance frome nearest one!!
   std::sort(human_pairs.begin(),human_pairs.end(),comparator);

   //for(size_t j(0);j<human_pairs.size();j++)
   //{
       //ROS_INFO("human pair index: distance = idx: %d, dist:  %.3lf", human_pairs[j].first, human_pairs[j].second);
   //}

   geometry_msgs::PoseArray sorted_human_belief;
   for(size_t idx(0);idx<candidate_size;idx++)
   {
        if(human_pairs[idx].second<6.0)
            sorted_human_belief.poses.push_back(human_candidates_poses.poses[human_pairs[idx].first]);
   }

   sorted_human_belief.header.frame_id ="map";
   sorted_human_belief.header.stamp = ros::Time::now();
   human_candidates_pub.publish(sorted_human_belief);
   
   //human_candidates_poses.header.frame_id ="map";
   //human_candidates_poses.header.stamp = ros::Time::now();
   //human_candidates_pub.publish(human_candidates_poses);
   */

}

void belief_manager::getCameraregion()
{

  double global_robot_x= global_pose[0];
  double global_robot_y= global_pose[1];
  double global_robot_theta = global_pose[2]+Head_Pos[0];

  visiblie_idx_set.clear();
  global_robot_theta=0.0;
  //Iteration for belief grid
  for(int i(0);i<camera_visible_region.info.width;i++)
    for(int j(0);j<camera_visible_region.info.height;j++)
  {
    int belief_map_idx=j*camera_visible_region.info.height+i;

    // double map_ogirin_x = camera_visible_region.info.origin.position.x+global_robot_x;
    // double map_ogirin_y = camera_visible_region.info.origin.position.y+global_robot_y;

    double map_ogirin_x = camera_visible_region.info.origin.position.x;
    double map_ogirin_y = camera_visible_region.info.origin.position.y;


    double trans_vector_x=(i+0.5)*camera_visible_region.info.resolution;
    double trans_vector_y=(j+0.5)*camera_visible_region.info.resolution;

    double rot_trans_vector_x = cos(global_robot_theta)*trans_vector_x-sin(global_robot_theta)*trans_vector_y;
    double rot_trans_vector_y = sin(global_robot_theta)*trans_vector_x+cos(global_robot_theta)*trans_vector_y;

    double belief_global_x=map_ogirin_x+rot_trans_vector_x;
    double belief_global_y=map_ogirin_y+rot_trans_vector_y;

    //solve
    bool line1_result =getlinevalue(1,belief_global_x,belief_global_y);
    bool line2_result =getlinevalue(2,belief_global_x,belief_global_y);


    if( line1_result && line2_result )
    {
      camera_visible_region.data[belief_map_idx]=30;  
      visiblie_idx_set.push_back(belief_map_idx);         //save cell_id 
    }
    else
      camera_visible_region.data[belief_map_idx]=0.0; 
  }



}

void belief_manager::publish_human_candidates(){
    
        //geometry_msgs::PoseArray poses_array_msg;

        //poses_array_msg.header.frame_id =target_frame;
        //poses_array_msg.header.stamp = ros::Time::now();

        //for(int i(0);i<input_poses.size();i++)
            //poses_array_msg.poses.push_back(input_poses[i]);

        //human_candidates_pub.publish(poses_array_msg);
}



bool belief_manager::getlinevalue(int line_type,double input_x, double input_y)
{

  double global_robot_theta = global_pose[2]+Head_Pos[0];
  // double global_robot_theta =Camera_angle;
  double theta_1=-FOVW*MATH_PI/180+global_robot_theta;
  double theta_2= FOVW*MATH_PI/180+global_robot_theta;
  
  double m_1=tan(theta_1);
  double m_2=tan(theta_2);

  int isspecial=0;

  if(theta_1<-MATH_PI/2.0 && theta_2 >-MATH_PI/2.0)
  {
    double temp=m_2;
    isspecial=1;
  }
  else if(theta_2> MATH_PI/2.0 && (theta_1 <MATH_PI/2.0))
  {
    isspecial=2;
  }
  else if (theta_1<-MATH_PI/2.0 && theta_2 <-MATH_PI/2.0)
  {
    isspecial=5;
  }
  else if(theta_2< -MATH_PI/2.0)
  {
    isspecial=3;
  }

  else if(theta_1>MATH_PI/2.0 && theta_2> MATH_PI/2.0)
  {
    isspecial=4;  
  }


   // std::cout<<"camera region section : "<<isspecial<<std::endl;
  
  double m=0.0;
  double coeff_sign=1.0;

  double global_robot_x= global_pose[0];
  double global_robot_y= global_pose[1];

  double res =0.0;

  switch(line_type){
  case 1:
      m=m_1;
      coeff_sign=-1.0;

      if(isspecial==0)
          coeff_sign=-1.0;
      else if(isspecial==1)
        coeff_sign=1.0;
      else if(isspecial==2)
        coeff_sign=-1.0;  
      else if(isspecial==4)
        coeff_sign=1.0; 
      else if(isspecial==5)
        coeff_sign=1.0; 

      break;
  case 2:
      m=m_2;
      coeff_sign=-1.0;
      if(isspecial==1)
        coeff_sign=1.0; 
      else if(isspecial==0)
        coeff_sign=1.0; 
      else if(isspecial==3)
        coeff_sign=1.0;
           

      break;
  default:
    std::cout<<"Wrong line type"<<std::endl;
      m=m_1;
    }

  res= m*input_x-m*global_robot_x+global_robot_y-input_y;

  if(res*coeff_sign>0 || res==0)
    return true;
  else
    return false;

}

void belief_manager::dyn_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    // Copy Data;
	// update dynamic map info
    dynamic_belief_map = *msg;
	//dynamic_belief_map.info.width  = msg->info.width;
	//dynamic_belief_map.info.height = msg->info.height;
	//dynamic_belief_map.info.resolution = msg->info.resolution;
	//dynamic_belief_map.info.origin.position.x = msg->info.origin.position.x;
	//dynamic_belief_map.info.origin.position.y =msg->info.origin.position.y;
	//dynamic_belief_map.data.resize(dynamic_belief_map.info.width*dynamic_belief_map.info.width);
	
    int dyn_occ_size = msg->data.size();
     m_dyn_occupancy.resize(dyn_occ_size);
	 for(int i(0);i<msg->data.size();i++)
	 {
	 	m_dyn_occupancy[i]=msg->data[i];
	 }
}

void belief_manager::scaled_dynamic_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{


}

void belief_manager::scaled_static_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("map Received");
    std::cout <<"static_Width: " << msg->info.width << std::endl;
    std::cout <<"static_Height: " << msg->info.height << std::endl;
    std::cout << "static_X origin:" << msg->info.origin.position.x << std::endl;
    std::cout << "static_Y origin:" << msg->info.origin.position.y << std::endl;
    std::cout <<"static_Resolution: " << msg->info.resolution << std::endl;   

    // Copy Data;
    Scaled_map = (*msg);
    Scaled_map.header.stamp =  ros::Time::now();
    Scaled_map.header.frame_id = "map";
 
}



bool belief_manager::check_staticObs(float x_pos,float y_pos)
{
  
  //return true if it is occupied with obstacles
  if (Scaled_map.data.size()>0)
  {   
      int obs_idx=globalcoord_To_SScaled_map_index(x_pos,y_pos);

    if(Scaled_map.data[obs_idx]>0)
        return true;
    else
      return false;
  }

}


bool belief_manager::check_cameraregion(float x_pos,float y_pos)
{

  if(abs(x_pos)<7.0 && abs(y_pos)<7.0)
  {
  //return true if it is occupied with obstacles
  if (camera_visible_region.data.size()>0)
  {   
    int obs_idx=CoordinateTransform_Global2_staticMap(x_pos,y_pos);
    
    if(obs_idx<camera_visible_region.data.size()){
      if(camera_visible_region.data[obs_idx]>10.0)
        return true;
      else
        return false;
    }
  }

  }

  return true;
}


int belief_manager::globalcoord_To_SScaled_map_index(float x_pos,float y_pos)
{
   std::vector<float> cur_coord(2,0.0);
  
   //for case of using static map
  float reference_origin_x =-4;
  float reference_origin_y =-4;
  float Grid_STEP=0.5;
  int num_grid=24;

  //for case of using static map
  // double reference_origin_x =-3.5;
  // double reference_origin_y =-3.5;
  float  temp_x  = x_pos-reference_origin_x;
  float  temp_y = y_pos-reference_origin_y;

  cur_coord[0]= (int) (temp_x/Grid_STEP);
  cur_coord[1]= (int)(temp_y/Grid_STEP);


  int robot_pos_id=num_grid*cur_coord[1]+cur_coord[0];
  //ROS_INFO("Robot pos ID : %d \n", robot_pos_id);

  return robot_pos_id;

}


double belief_manager::getDistance_from_Vec(std::vector<double> origin, double _x, double _y)
{
  double temp=0.0;

  temp=(origin[0]-_x)*(origin[0]-_x);
  temp+=(origin[1]-_y)*(origin[1]-_y);
  temp=sqrt(temp);

  return temp;
}

void belief_manager::global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

   global_pose[0]=msg->pose.position.x;
   global_pose[1]=msg->pose.position.y;

   tf::StampedTransform baselinktransform;
   listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(2.0));
   listener.lookupTransform("map", "base_link", ros::Time(0), baselinktransform);
   double yaw_tf =   tf::getYaw(baselinktransform.getRotation()); 

    global_pose[2]=yaw_tf;
}

void belief_manager::target_poses_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{

    isTargetDetected=true;
    //ROS_INFO("taget poses callback---");

    num_of_detected_target=msg->poses.size();
    index_of_target_occ_cells_updated_recently.clear();

    if(num_of_detected_target>0)
       cur_target.resize(num_of_detected_target);
    else
    {
      update_human_occ_belief(NO_HUMANS_DETECTED);
      return;
    }

    for(int i(0);i<num_of_detected_target;i++)
    {
      double cur_target_x=msg->poses[i].position.x;
      double cur_target_y=msg->poses[i].position.y;

      cur_target[i].resize(2,0.0);
      cur_target[i][0]=cur_target_x;
      cur_target[i][1]=cur_target_y;

      //ROS_INFO("---human poses callback---");
      int target_mapidx=CoordinateTransform_Global2_beliefMap(cur_target_x,cur_target_y);
      //if((target_mapidx<0) || (target_mapidx>10000))
          //return;

      if((target_mapidx<Human_Belief_Scan_map.data.size())&& (target_mapidx>0))
      {
          index_of_target_occ_cells_updated_recently.push_back(target_mapidx);
          Human_Belief_Scan_map.data[target_mapidx] = 80;
          Human_Belief_type_map.data[target_mapidx] = HUMAN_OCCUPIED;

         if(map_index_of_target_cells_to_prob.count(target_mapidx) >0 ){
          // Encountered the same cells. Update probability:
          // P(H|S) = P(S|H)P(H) / P(S)
          //std::cout<<"human map idx: " <<target_mapidx <<std::endl;
          float prior = map_index_of_target_cells_to_prob[target_mapidx]; // P(H)
          //std::cout<<"prior : " <<prior <<std::endl;
          float P_S = P_S_given_H*prior + P_S_given_Hc*(1-prior);
          //std::cout<<"P_S : " <<P_S <<std::endl;
          float posterior = (P_S_given_H)*prior / P_S;
          //std::cout<<"posterior : " <<posterior <<std::endl;
          //
          //
          //ROS_INFO("----------index: %d", target_mapidx);
          map_index_of_target_cells_to_prob[target_mapidx] = posterior;

          }else{
            map_index_of_target_cells_to_prob[target_mapidx] = 0.05;
          }		
      }
      //human_occupied_idx.push_back(target_mapidx);
   }

    //ROS_INFO("updatd--------");
    update_human_occ_belief(HUMANS_DETECTED);
}

/**
 * @brief Find nearest cell of a specified value
 * @param result Index of located cell
 * @param start Index initial cell to search from
 * @param val Specified value to search for
 * @param costmap Reference to map data
 * @return True if a cell with the requested value was found
 */

std::vector<unsigned int> belief_manager::nhood4(unsigned int idx){
    //get 4-connected neighbourhood indexes, check for edge of map
    std::vector<unsigned int> out;

    //unsigned int search_size =Target_Search_map.info.width*Target_Search_map.info.height;
    unsigned int size_x_ = costmap_size_x;
    unsigned int size_y_ = costmap_size_y;

    if (idx > size_x_ * size_y_ -1){
        ROS_WARN("Evaluating nhood for offmap point");
        return out;
    }

    if(idx % size_x_ > 0){
        out.push_back(idx - 1);
    }
    if(idx % size_x_ < size_x_ - 1){
        out.push_back(idx + 1);
    }
    if(idx >= size_x_){
        out.push_back(idx - size_x_);
    }
    if(idx < size_x_*(size_y_-1)){
        out.push_back(idx + size_x_);
    }
    return out;

}

std::vector<unsigned int> belief_manager::nhood8(unsigned int idx){
    //get 8-connected neighbourhood indexes, check for edge of map
    std::vector<unsigned int> out = nhood4(idx);

    unsigned int size_x_ = costmap_size_x;
    unsigned int size_y_ = costmap_size_y;

    if (idx > size_x_ * size_y_ -1){
        return out;
    }

    if(idx % size_x_ > 0 && idx >= size_x_){
        out.push_back(idx - 1 - size_x_);
    }
    if(idx % size_x_ > 0 && idx < size_x_*(size_y_-1)){
        out.push_back(idx - 1 + size_x_);
    }
    if(idx % size_x_ < size_x_ - 1 && idx >= size_x_){
        out.push_back(idx + 1 - size_x_);
    }
    if(idx % size_x_ < size_x_ - 1 && idx < size_x_*(size_y_-1)){
        out.push_back(idx + 1 + size_x_);
    }

    return out;
}


bool belief_manager::nearestCell(unsigned int &result, unsigned int start, unsigned char val){

    const unsigned char* map_ = getCharMap();
    //const unsigned int size_x = costmap.getSizeInCellsX(), size_y = costmap.getSizeInCellsY();

    unsigned int search_size =Target_Search_map.info.width*Target_Search_map.info.height;

    if(start >= search_size){
        return false;
    }

    //initialize breadth first search
    std::queue<unsigned int> bfs;
    std::vector<bool> visited_flag(search_size, false);

    //push initial cell
    bfs.push(start);
    visited_flag[start] = true;

    //search for neighbouring cell matching value
    while(!bfs.empty()){
        unsigned int idx = bfs.front();
        bfs.pop();

        //return if cell of correct value is found
        if(map_[idx] == val){
            result = idx;
            return true;
        }

        //iterate over all adjacent unvisited cells
        BOOST_FOREACH(unsigned nbr, nhood8(idx)){
            if(!visited_flag[nbr]){
                bfs.push(nbr);
                visited_flag[nbr] = true;
            }
        }
    }

    return false;
}


unsigned char* belief_manager::getCharMap() const
{
    return costmap_;
}

std::vector<frontier_exploration::Frontier> belief_manager::searchFrom(geometry_msgs::Point position)
{
    std::vector<frontier_exploration::Frontier> frontier_list;

    unsigned int target_mapidx=(unsigned int) CoordinateTransform_Global2_beliefMap(global_pose[0], global_pose[1]);
    unsigned int search_size =Target_Search_map.info.width*Target_Search_map.info.height;
     //initialize flag arrays to keep track of visited and frontier cells
    std::vector<bool> frontier_flag(search_size, false);
    std::vector<bool> visited_flag(search_size, false);

    //initialize breadth first search
    std::queue<unsigned int> bfs;

    unsigned int clear, pos = target_mapidx;
    if(nearestCell(clear, pos, NO_INFORMATION)){
        bfs.push(clear);
    }else{
        bfs.push(pos);
        ROS_WARN("Could not find nearby clear cell to start search");
    }
    visited_flag[bfs.front()] = true;

    while(!bfs.empty()){
        unsigned int idx = bfs.front();
        bfs.pop();

        //iterate over 4-connected neighbourhood
        BOOST_FOREACH(unsigned nbr, nhood4(idx)){
            //add to queue all free, unvisited cells, use descending search in case initialized on non-free cell
            if(costmap_[nbr] <= costmap_[idx] && !visited_flag[nbr]){
                visited_flag[nbr] = true;
                bfs.push(nbr);
                //check if cell is new frontier cell (unvisited, NO_INFORMATION, free neighbour)
            }else if(isNewFrontierCell(nbr, frontier_flag)){
                frontier_flag[nbr] = true;
                frontier_exploration::Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
                if(new_frontier.size > min_frontier_size_){
                    frontier_list.push_back(new_frontier);
                }
            }
        }
    }

    for(auto& frontier : frontier_list) {
        frontier.cost = frontierCost(frontier);
    }
    std::sort(frontier_list.begin(), frontier_list.end(),[](const frontier_exploration::Frontier& f1, const frontier_exploration::Frontier& f2) { return f1.cost < f2.cost; });
    return frontier_list;

}

bool belief_manager::isNewFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag){

    //map_ = costmap_.getCharMap();
    //check that cell is unknown and not already marked as frontier
    if(costmap_[idx] != NO_INFORMATION || frontier_flag[idx]){
        return false;
    }

    //frontier cells should have at least one cell in 4-connected neighbourhood that is free
    BOOST_FOREACH(unsigned int nbr, nhood4(idx)){
        if(costmap_[nbr] == FREE_SPACE){
            return true;
        }
    }

    return false;
}

double belief_manager::frontierCost(const frontier_exploration::Frontier& frontier)
{
  return (potential_scale_ * frontier.min_distance *map_res) -
         (gain_scale_ * frontier.size * map_res);
}


frontier_exploration::Frontier belief_manager::buildNewFrontier(unsigned int initial_cell, unsigned int reference_, std::vector<bool>& frontier_flag){

    //initialize frontier structure
    frontier_exploration::Frontier output;
    //geometry_msgs::Point centroid, middle;

    output.centroid.x = 0;
    output.centroid.y = 0;
    output.size = 1;
    output.min_distance = std::numeric_limits<double>::infinity();

    //record initial contact point for frontier
    //get glboal coordinate from map coordinate
    std::vector<double> temp_pose(2,0.0);
    Mapidx2GlobalCoord(initial_cell,temp_pose);
    output.travel_point.x=temp_pose[0];
    output.travel_point.y=temp_pose[1];

    //push initial gridcell onto queue
    std::queue<unsigned int> bfs;
    bfs.push(initial_cell);

    std::vector<double> temp_referencepose(2,0.0);
    Mapidx2GlobalCoord(reference_,temp_referencepose);
    double reference_x=temp_referencepose[0];
    double reference_y=temp_referencepose[1];

    while(!bfs.empty()){
        unsigned int idx = bfs.front();
        bfs.pop();

        //try adding cells in 8-connected neighborhood to frontier
        BOOST_FOREACH(unsigned int nbr, nhood8(idx)){
            //check if neighbour is a potential frontier cell
            if(isNewFrontierCell(nbr,frontier_flag)){

                //mark cell as frontier
                frontier_flag[nbr] = true;
                std::vector<double> temp_position(2,0.0);
                Mapidx2GlobalCoord(nbr,temp_position);
                double wx = temp_position[0];
                double wy = temp_position[1];
                //unsigned int mx,my;
                //double wx,wy;
                //costmap_.indexToCells(nbr,mx,my);
                //costmap_.mapToWorld(mx,my,wx,wy);

                geometry_msgs::Point point;
                point.x = wx;
                point.y = wy;
                output.points.push_back(point);
                //update frontier size
                output.size++;

                //update centroid of frontier
                output.centroid.x += wx;
                output.centroid.y += wy;

                //determine frontier's distance from robot, going by closest gridcell to robot
                double distance = sqrt(pow((double(reference_x)-double(wx)),2.0) + pow((double(reference_y)-double(wy)),2.0));
                if(distance < output.min_distance){
                    output.min_distance = distance;
                    output.middle.x = wx;
                    output.middle.y = wy;
                }

                //add to queue for breadth first search
                bfs.push(nbr);
            }
        }
    }

    //average out frontier centroid
    output.centroid.x /= output.size;
    output.centroid.y /= output.size;
    
    output.centroid.x+=0.25;
    output.centroid.y+=0.25;

    //output.travel_point = output.centroid;

    //if(travel_point_ == "closest"){
         //point already set
    //}else if(travel_point_ == "middle"){
        //output.travel_point = output.middle;
    //}else if(travel_point_ == "centroid"){
        //output.travel_point = output.centroid;
    //}else{
        //ROS_ERROR("Invalid 'frontier_travel_point' parameter, falling back to 'closest'");
         //point already set
    //}

    return output;
}
void belief_manager::getNextFrontiers(const std::vector<frontier_exploration::Frontier>& frontiers)
{

    frontier_exploration::Frontier selected;
    selected.min_distance = std::numeric_limits<double>::infinity();

    //pointcloud for visualization purposes
    pcl::PointCloud<pcl::PointXYZI> frontier_cloud_viz;
    pcl::PointXYZI frontier_point_viz(50);
    int max_;

    BOOST_FOREACH(frontier_exploration::Frontier frontier, frontiers){
        //load frontier into visualization poitncloud
        frontier_point_viz.x = frontier.travel_point.x;
        frontier_point_viz.y = frontier.travel_point.y;
        frontier_cloud_viz.push_back(frontier_point_viz);

        //ROS_INFO("list travel points:  %.3lf, %.3lf", frontier.travel_point.x, frontier.travel_point.y);
        //ROS_INFO("min_distance : %.3lf", frontier.min_distance);
        //check if this frontier is the nearest to robot
        if (frontier.min_distance < selected.min_distance){
            selected = frontier;
            max_= frontier_cloud_viz.size()-1;
            }
        }
        //ROS_INFO("max: %d", max);
        //ROS_INFO("list travel points - %.3lf, %.3lf", frontier.travel_point.x, frontier,travel_point.y);

        if (std::isinf(selected.min_distance)) {
            ROS_INFO("No valid (non-blacklisted) frontiers found, exploration complete");
            ROS_DEBUG("No valid (non-blacklisted) frontiers found, exploration complete");
    }

   //color selected frontier
   frontier_cloud_viz[max_].intensity = 100;

   //publish visualization point cloud-----------------------------
   sensor_msgs::PointCloud2 frontier_viz_output;
   pcl::toROSMsg(frontier_cloud_viz,frontier_viz_output);
   frontier_viz_output.header.frame_id = "map";
   frontier_viz_output.header.stamp = ros::Time::now();
   frontier_cloud_pub.publish(frontier_viz_output);
   //---------------------------------------------------------------

   //set goal pose to next frontier
   geometry_msgs::PoseStamped next_frontier;
   next_frontier.header.frame_id = "map";
   next_frontier.header.stamp = ros::Time::now();

   next_frontier.pose.position = selected.travel_point;
   //next_frontier.pose.orientation = tf::createQuaternionMsgFromYaw( yawOfVector(start_pose.pose.position, next_frontier.pose.position) );
   nextfrontier_pose_pub.publish(next_frontier);


}

void belief_manager::visualizeFrontiers(const std::vector<frontier_exploration::Frontier>& frontiers)
{
        std_msgs::ColorRGBA blue;
        blue.r = 0;
        blue.g = 0;
        blue.b = 1.0;
        blue.a = 1.0;
        std_msgs::ColorRGBA red;
        red.r = 1.0;
        red.g = 0;
        red.b = 0;
        red.a = 1.0;
        std_msgs::ColorRGBA green;
        green.r = 0;
        green.g = 1.0;
        green.b = 0;
        green.a = 1.0;

        ROS_DEBUG("visualising %lu frontiers", frontiers.size());
        visualization_msgs::MarkerArray markers_msg;
        std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
        visualization_msgs::Marker m;

        m.header.frame_id ="map";
        m.header.stamp = ros::Time::now();
        m.ns = "frontiers";
        m.scale.x = 1.0;
        m.scale.y = 1.0;
        m.scale.z = 1.0;
        m.color.r = 0;
        m.color.g = 0;
        m.color.b = 255;
        m.color.a = 255;
        // lives forever
        m.lifetime = ros::Duration(0);
        m.frame_locked = true;

        // weighted frontiers are always sorted
        double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;
        m.action = visualization_msgs::Marker::ADD;
        size_t id = 0;
        for (auto& frontier : frontiers) {
            m.type = visualization_msgs::Marker::POINTS;
            m.id = int(id);
            m.pose.position = {};
            m.scale.x = 0.1;
            m.scale.y = 0.1;
            m.scale.z = 0.1;
            m.points = frontier.points;
            m.color=red;
            //if (goalOnBlacklist(frontier.centroid)) {
                //m.color = red;
            //} else {
                //m.color = blue;
            //}
            markers.push_back(m);
            ++id;
            m.type = visualization_msgs::Marker::SPHERE;
            m.id = int(id);
            m.pose.position = frontier.travel_point;
            // scale frontier according to its cost (costier frontiers will be smaller)
            double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
            m.scale.x = scale;
            m.scale.y = scale;
            m.scale.z = scale;
            m.points = {};
            m.color = green;
            markers.push_back(m);
            ++id;
        }
        size_t current_markers_count = markers.size();

        // delete previous markers, which are now unused
        m.action = visualization_msgs::Marker::DELETE;
        for (; id < last_markers_count_; ++id) {
            m.id = int(id);
            markers.push_back(m);
        }

        last_markers_count_ = current_markers_count;
        //marker_array_publisher_.publish(markers_msg);
        frontier_marker_pub.publish(markers_msg);
    }




void belief_manager::human_poses_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{

    //ROS_INFO("human poses callback");
    index_of_target_occ_cells_updated_recently.clear();

    //check number of detected_humans
    num_of_detected_human=msg->poses.size();

    if(num_of_detected_human>0)
       cur_target.resize(num_of_detected_human);
    else
    {
      update_human_occ_belief(NO_HUMANS_DETECTED);
      return;
    }


    ROS_INFO("human poses callback---");
    for(int i(0);i<num_of_detected_human;i++)
    {
      double cur_people_x=msg->poses[i].position.x;
      double cur_people_y=msg->poses[i].position.y;

      cur_target[i].resize(2,0.0);
      cur_target[i][0]=cur_people_x;
      cur_target[i][1]=cur_people_y;

      ROS_INFO("---human poses callback---");
      int target_mapidx=CoordinateTransform_Global2_beliefMap(cur_people_x,cur_people_y);
      //if((target_mapidx<0) || (target_mapidx>10000))
          //return;

      if((target_mapidx<Human_Belief_Scan_map.data.size())&& (target_mapidx>0))
      {
          index_of_target_occ_cells_updated_recently.push_back(target_mapidx);
          Human_Belief_Scan_map.data[target_mapidx] = 80;
          Human_Belief_type_map.data[target_mapidx] = HUMAN_OCCUPIED;

         if(map_index_of_target_cells_to_prob.count(target_mapidx) >0 ){
          // Encountered the same cells. Update probability:
          // P(H|S) = P(S|H)P(H) / P(S)
          //std::cout<<"human map idx: " <<target_mapidx <<std::endl;
          float prior = map_index_of_target_cells_to_prob[target_mapidx]; // P(H)
          //std::cout<<"prior : " <<prior <<std::endl;
          float P_S = P_S_given_H*prior + P_S_given_Hc*(1-prior);
          //std::cout<<"P_S : " <<P_S <<std::endl;
          float posterior = (P_S_given_H)*prior / P_S;
          //std::cout<<"posterior : " <<posterior <<std::endl;
          //
          //
          ROS_INFO("---------------human poses callback---");
          map_index_of_target_cells_to_prob[target_mapidx] = posterior;

          }else{
            map_index_of_target_cells_to_prob[target_mapidx] = 0.05;
          }		
      }
      //human_occupied_idx.push_back(target_mapidx);
   }

    ROS_INFO("human poses callback--------");
    update_human_occ_belief(HUMANS_DETECTED);
   // printf("size yolo : %d \n",cur_target.size());
}


void belief_manager::update_human_occ_belief(int update_type){

    std::map<int, int> map_index_recently_updated;
    if (update_type == (int) HUMANS_DETECTED){
        for(int i = 0; i < index_of_target_occ_cells_updated_recently.size(); i++){
            int index = index_of_target_occ_cells_updated_recently[i];
            map_index_recently_updated[index] = index;
            //std::cout<<"map_index_recently updated set :" <<index <<std::endl;
        }
    }

    // Extract_world_indices_from_visible_camera_region(float depth, float width, float res)
    // For each cell, check if is labeled as human.
    // If labeled as human, update probability of cell regions
    std::vector<int> indices_to_assign_as_free;

    //std::cout<<"camera visiblie_idx_set size : "<<visiblie_idx_set.size()<<std::endl;
    for(int i(0);i< visiblie_idx_set.size();i++)
    {
        int cell_idx= visiblie_idx_set[i];
        if (map_index_recently_updated.count(cell_idx) == 1){
            continue;
            std::cout<<"continue "<<cell_idx<<std::endl;
        }
        if(Human_Belief_type_map.data[cell_idx]==HUMAN_OCCUPIED)
        {
            float prior = map_index_of_target_cells_to_prob[cell_idx]; // P(H)
            //float P_S = P_Sc_given_H*(prior) + P_Sc_given_Hc*(1-prior);
            float posterior = prior*0.5;
            map_index_of_target_cells_to_prob[cell_idx] =posterior;

            if (posterior < PROB_THRESH){
                indices_to_assign_as_free.push_back(cell_idx);
                //std::cout << "free index : "<<cell_idx<< ", Prob: " << posterior << std::endl;
            }
        }
        else
        {
        
            indices_to_assign_as_free.push_back(cell_idx);
        
        }

    }
        for(size_t i = 0; i < indices_to_assign_as_free.size(); i++){
            int index_to_erase =  indices_to_assign_as_free[i];
            map_index_of_target_cells_to_prob.erase(index_to_erase);
            Human_Belief_Scan_map.data[index_to_erase]=0.00;
            Human_Belief_type_map.data[index_to_erase]=0;
            Target_Search_map.data[index_to_erase]=0.0;
            costmap_[index_to_erase]=FREE_SPACE;
        }
}



// callback for messages
int belief_manager::CoordinateTransform_Global2_staticMap(float global_x, float global_y)
{
  double reference_origin_x=camera_visible_region.info.origin.position.x;
  double reference_origin_y=camera_visible_region.info.origin.position.y;

  //Find the coordinate w.r.t map origin
  double  temp_x  = global_x - reference_origin_x;
  double  temp_y  = global_y - reference_origin_y;

  //Find the map cell idx for x, y
  std::vector<int> human_coord(2,0);
  human_coord[0]= (int) (temp_x/camera_visible_region.info.resolution);
  human_coord[1]= (int) (temp_y/camera_visible_region.info.resolution);

  //Find the map index from cell x, y
  int static_map_idx= human_coord[0]+camera_visible_region.info.width*human_coord[1];

  // std::cout<<"map_idx : "<<static_map_idx<<std::endl;
  return static_map_idx;
   
}


int belief_manager::CoordinateTransform_Global2_beliefMap(double global_x, double global_y)
{

	double reference_origin_x=Human_Belief_Scan_map.info.origin.position.x;
	double reference_origin_y=Human_Belief_Scan_map.info.origin.position.y;

	//Find the coordinate w.r.t map origin
	double  temp_x  = global_x - reference_origin_x;
	double  temp_y  = global_y - reference_origin_y;

	//Find the map cell idx for x, y
	std::vector<int> human_coord(2,0);
 	human_coord[0]= (int) (temp_x/Human_Belief_Scan_map.info.resolution);
 	human_coord[1]= (int) (temp_y/Human_Belief_Scan_map.info.resolution);

 	//Find the map index from cell x, y
 	int static_map_idx= human_coord[0]+Human_Belief_Scan_map.info.width*human_coord[1];

 	return static_map_idx;

}


bool belief_manager::Comparetwopoistions(std::vector<double> pos,std::vector<double> pos2, double criterion)
{
  //return true if there are in criterion distance 
  double temp_dist=0.0;
  for(int i(0);i<2;i++) 
  {
    temp_dist+=pow((pos[i]-pos2[i]),2);
  }

  temp_dist=sqrt(temp_dist);

  if(temp_dist<criterion)
    return true;
  

  return false;
}

void belief_manager::executeCB(const visual_perception::SearchGoalConstPtr &goal)
  {
      double ros_rate = 2;
      ros::Rate r(ros_rate);

      isActionActive=true;
      reset_search_map();
      bool success = true;
     // ROS_INFO("%s: succeeded",action_name_.c_str());
     if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        // break;
      }

     while(ros::ok() && isActionActive&& !as_.isPreemptRequested())
     {
        //process_target(navtarget_pose[0],navtarget_pose[1],navtarget_pose[2]);
        //Sending_velcmd();
        //
        boost::mutex::scoped_lock lock(filter_mutex_);
        geometry_msgs::Point start_pose;
        start_pose.x=global_pose[0];
        start_pose.y=global_pose[1];

        auto frontier_list = searchFrom(start_pose);

        if(frontier_list.size() == 0){
            ROS_INFO("No frontiers found, exploration complete");
            isActionActive=false;
            result_.success = true;
            as_.setSucceeded(result_);
            return;
        }
        else
        {
            ROS_INFO("number of frontiers found: %d", frontier_list.size());
            visualizeFrontiers(frontier_list);
            getNextFrontiers(frontier_list);
        }

        //searchfrom
        lock.unlock();

        as_.publishFeedback(feedback_);
    	ros::spinOnce();
        r.sleep();
     }

     //as_.setSucceeded(result_);
}




// filter loop
void belief_manager::spin()
{
  //ROS_INFO("People tracking manager started.");

  while (ros::ok())
  {
    publish_cameraregion();
    publish_target_belief();
    publish_searchmap();

    if(!isTargetDetected)
        update_human_occ_belief(NO_HUMANS_DETECTED);

    // ------ LOCKED ------
    /*
    boost::mutex::scoped_lock lock(filter_mutex_);
    geometry_msgs::Point start_pose;
    start_pose.x=global_pose[0];
    start_pose.y=global_pose[1];


    if(isActionActive){
        auto frontier_list = searchFrom(start_pose);

        if(frontier_list.size() == 0){
            ROS_INFO("No frontiers found, exploration complete");
            
        }
        else
        {
            ROS_INFO("number of frontiers found: %d", frontier_list.size());

            visualizeFrontiers(frontier_list);
            getNextFrontiers(frontier_list);
        }
    }

    //searchfrom
    lock.unlock();
    */
    // ------ LOCKED ------
    // sleep
    ros::Duration(0.5).sleep();

    ros::spinOnce();
  }
};

// ----------
// -- MAIN --
// ----------
int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "search_server");
  belief_manager search_manager(ros::this_node::getName());

  search_manager.spin();
  // Clean up

  return 0;
}
