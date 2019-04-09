#include "human_belief.h"

using namespace std;
using namespace tf;
using namespace message_filters;

static const double       sequencer_delay            = 0.5; //TODO: this is probably too big, it was 0.8
static const unsigned int sequencer_internal_buffer  = 100;
static const unsigned int sequencer_subscribe_buffer = 10;
static const unsigned int num_particles_tracker      = 1000;
static const double       tracker_init_dist          = 4.0;

bool IsNotInitilized = true;


// constructor
belief_manager::belief_manager(std::string name)
  :as_(nh_, name, boost::bind(&belief_manager::executeCB, this,_1), false),
   action_name_(name),
   robot_state_()
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
  for(int k(0);k<belief_size;k++)
      Target_Search_map.data[k]=0.0;


  camera_visible_region_pub=nh_.advertise<nav_msgs::OccupancyGrid>("/camera_region_map", 10, true);
  human_candidates_pub=nh_.advertise<geometry_msgs::PoseArray>("/human_belief_pose_array", 10, true);
  belief_pub=nh_.advertise<nav_msgs::OccupancyGrid>("/human_belief_map", 10, true);
  searchmap_pub=nh_.advertise<nav_msgs::OccupancyGrid>("/target_search_map", 10, true);

  joint_state_sub =nh_.subscribe<sensor_msgs::JointState>("/hsrb/joint_states", 10, &belief_manager::joint_states_callback,this);
  target_poses_sub =nh_.subscribe<geometry_msgs::PoseArray>("/bottle_poses", 10, &belief_manager::target_poses_callback,this);
  globalpose_sub=nh_.subscribe<geometry_msgs::PoseStamped>("/global_pose",10,&belief_manager::global_pose_callback,this);
 
}

// destructor
belief_manager::~belief_manager()
{
  // delete sequencer
  // delete all trackers
};


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
            Target_Search_map.data[index_to_erase]=100;
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
     // ros::Rate r(1);
     //result_.is_free = false;
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

     //bool isObstacle=false;
     //geometry_msgs::PointStamped point_in;
     //geometry_msgs::PointStamped point_out;
     //if(goal->pose.header.frame_id!="map")
     //{
        //change reference frame if it is not map frame
        //ROS_INFO("goal target reference frame : %s", goal->pose.header.frame_id.c_str());
        //listener.waitForTransform("map",goal->pose.header.frame_id, ros::Time(0), ros::Duration(3.0));
        //goal->pose.header.frame_id = "map";
        //listener.transformPoint("map", goal->pose, point_out);
        //isObstacle=check_obstacle(point_out.point.x, point_out.point.y);
     //}
     //else
     //{
        //check obstacle information w.r.t map frame
        //ROS_INFO("map is goal target reference frame ");
        //isObstacle=check_obstacle(goal->pose.point.x, goal->pose.point.y);
     //}

     //ROS_INFO("is obstacle? %d",isObstacle );
     //result_.is_free = isObstacle;
     as_.setSucceeded(result_);
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


    // ------ LOCKED ------
    boost::mutex::scoped_lock lock(filter_mutex_);
    lock.unlock();
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
