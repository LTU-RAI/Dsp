#include "dsl_gridsearch/dsl_grid3d.h"

#include <visualization_msgs/Marker.h>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>

#include <chrono>
#include <iostream>
#include <fstream>
#include <math.h>

namespace dsl_gridsearch
{

DslGrid3D::DslGrid3D(ros::NodeHandle nh, ros::NodeHandle nh_private) :
  nh_(nh),
  nh_private_(nh_private)
{
  ROS_INFO("DslGrid3D::DslGrid3D(ros::NodeHandle nh, ros::NodeHandle nh_private)");        
  double grid_xmin, grid_ymin, grid_zmin;

  if (!nh_private_.getParam ("cells_per_meter", cells_per_meter_))
    cells_per_meter_ = 1.;
  if (!nh_private_.getParam ("spline_step_", spline_step_))
    spline_step_ = .1;
  if (!nh_private_.getParam ("grid_length", grid_length_))
    grid_length_ = -1;
  if (!nh_private_.getParam ("grid_width", grid_width_))
    grid_width_ = -1;
  if (!nh_private_.getParam ("grid_height", grid_height_))
    grid_height_ = -1;
  if (!nh_private_.getParam ("grid_xmin", grid_xmin))
    grid_xmin = 0;
  if (!nh_private_.getParam ("grid_ymin", grid_ymin))
    grid_ymin = 0;
  if (!nh_private_.getParam ("grid_zmin", grid_zmin))
    grid_zmin = 0;

  if (!nh_private_.getParam ("use_gazebo_odom", use_gazebo_odom_))
    use_gazebo_odom_ = false;
    if (!nh_private_.getParam ("odom_topic", odom_topic_))
        odom_topic_ = "/pixy/truth/NWU";
/*  if (!nh_private_.getParam ("spline_step_", spline_step_))
    spline_step_ = .1;*/
  if (!nh_private_.getParam ("odom_frame_id", odom_frame_id_))
    odom_frame_id_ = "odom";
  if (!nh_private_.getParam ("unknown_value", DSL_UNKNOWN))
    DSL_UNKNOWN = 10000;

  occ_map_viz_pub_ = nh_.advertise<visualization_msgs::Marker>( "/dsl_grid3d/occupancy_map",  0);
  path_pub_ = nh_.advertise<nav_msgs::Path>( "/dsl_grid3d/path",  0);
  optpath_pub_ = nh_.advertise<nav_msgs::Path>( "/dsl_grid3d/optpath",  0);
  splinepath_pub_ = nh_.advertise<nav_msgs::Path>( "/dsl_grid3d/splinepath",  0);
  splineoptpath_pub_ = nh_.advertise<nav_msgs::Path>( "/dsl_grid3d/splineoptpath",  0);

  if(use_gazebo_odom_)
  {
    ROS_INFO("Using odom ass start");
    set_start_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic_, 1, 
      &DslGrid3D::handleSetStartOdom, this);
  }
  else
  {
    ROS_INFO("Manualy set start");
    set_start_sub_ = nh_.subscribe<geometry_msgs::Point>("/dsl_grid3d/set_start", 1, 
      &DslGrid3D::handleSetStart, this);
  }

  set_goal_sub_ = nh_.subscribe<geometry_msgs::Point>("/dsl_grid3d/set_goal", 1,
    &DslGrid3D::handleSetGoal, this);
  set_frontier_sub = nh_.subscribe<exploration::Frontier>("/next_frontier", 1,
    &DslGrid3D::handleSetFrontier, this);
  set_occupied_sub_ = nh_.subscribe<geometry_msgs::Point>("/dsl_grid3d/set_occupied", 1, 
    &DslGrid3D::handleSetOccupied, this);
  set_unoccupied_sub_ = nh_.subscribe<geometry_msgs::Point>("/dsl_grid3d/set_unoccupied", 1, 
    &DslGrid3D::handleSetUnoccupied, this);
  //get_octomap_sub_ = nh_.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1,
  get_octomap_sub_ = nh_.subscribe<octomap_msgs::Octomap>("/octomap_full", 1,
    &DslGrid3D::octomap_data_callback, this);


  //timer = nh_private_.createTimer(ros::Duration(5.0), &DslGrid3D::spin, this);
  //ROS_INFO("Spinner started");

  start_pos << 0,0,0; 
  goal_pos << 0,0,0; 
}

void DslGrid3D::octomap_data_callback(const octomap_msgs::OctomapConstPtr& msg)
{	
  //ROS_INFO("DslGrid3D::octomap_data_callback(const octomap_msgs::OctomapConstPtr& msg)");        
  //using namespace std::chrono;
  //high_resolution_clock::time_point t1 = high_resolution_clock::now();


    // This can be cast to other types. An octree seems to be appropriate for publishing
    std::shared_ptr<octomap::OcTree> tree(dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg)));
/*	std::cout << "resolution: " << tree->getResolution() << std::endl;
	std::cout << "type: " << tree->getTreeType() << std::endl;
	std::cout << "size: " << tree->size() << std::endl;
*/
    res_octomap = tree->getResolution();
    double length_test, width_test, height_test;
    tree->getMetricSize(length_test, width_test, height_test);
    tree->getMetricMin(pmin(0), pmin(1), pmin(2));
    tree->getMetricMax(pmax(0), pmax(1), pmax(2));

//    ROS_INFO("l %f, w %f, h %f\nl %f, w %f, h %f", length_test, width_test, height_test, length, width, height);
    if (length_test * width_test * height_test == length * width * height)
    {
        updateGDSL(tree);
    }
    else
    {

        length = length_test;
        width = width_test;
        height = height_test;
        buildGDSL(tree);
        grid_built = true;
    }
    publishOccupancyGrid();
    setAndPublishPath();
}

void DslGrid3D::buildGDSL(std::shared_ptr<octomap::OcTree> tree)
{
  //ROS_INFO("DslGrid3D::buildGDSL(std::shared_ptr<octomap::OcTree> tree)");
  //using namespace std::chrono;
  //high_resolution_clock::time_point t1 = high_resolution_clock::now();
    int count = 0;
//    std::cout << "Metric size. length: " << length/res_octomap << " width: " << width/res_octomap << " height: " << height/res_octomap << std::endl;
    length_metric = length/res_octomap;
    width_metric = width/res_octomap;
    height_metric = height/res_octomap;

    //const double occupied_val = DSL_OCCUPIED;
    int size = length_metric * width_metric * height_metric;
//    std::cout<<"Size: "<<size<<std::endl;
    occupancy_map.reset(new double[size]);

   for(int i = 0; i < size; i++)
   {
     occupancy_map[i] = DSL_UNKNOWN; 
   }

   for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it)
    {
        Eigen::Vector3d pos(it.getX(), it.getY(), it.getZ());
        pos = posRes(pos);
        int n = it.getSize() / res_octomap;
        int i = -n/2;
	it->getOccupancy();
        do{
            int j = -n/2;
            do{
                int k = -n/2;
                do{
	            int idx = ((int) pos(0) + i) + ((int) pos(1) + j) *length_metric + ((int) pos(2) + k) *length_metric*width_metric;
	            if(tree->isNodeOccupied(*it)) {
                        Eigen::Vector3d p(pos(0) + i, pos(1) + j, pos(2) + k);
                        saftyMarginal(p, false);
		        occupancy_map[idx] = DSL_OCCUPIED;
	            }
                    else if (occupancy_map[idx] >= DSL_UNKNOWN){
                        occupancy_map[idx] = occupancy_map[idx] - DSL_UNKNOWN + 1;
                    }
                    k++;
                } while (k < n/2);
                j++;
            } while (j < n/2); 
            i++;
        } while ( i < n/2);
    }

    //std::cout << "count: " << std::endl;

    //Bound occupancy_map from top and bottom
/*	for(int x = 0; x < ogrid_->getLength(); x++)
	{  
        for(int y = 0; y < width_metric; y++)  
		{
			//for(int z = 0; z < height_metric; z++)  
			//{
				Eigen::Vector3i gpMin(x, y, zmin);
                Eigen::Vector3d wpMin = ogrid_->gridToPosition(gpMin);
				ogrid_->setOccupied(wpMin, true);

				Eigen::Vector3i gpMax(x, y, height_metric-1);
                Eigen::Vector3d wpMax = ogrid_->gridToPosition(gpMax);
				ogrid_->setOccupied(wpMax, true);
			//}
        }
	}
*/
//	std::cout << "Grid Bounds: " << pmin.transpose() << " and " 
//    << pmax.transpose() << std:: endl;

  //high_resolution_clock::time_point t2 = high_resolution_clock::now();
  //duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
/*  std::cout << "----LOG: Build OccupancyGrid. It took me " << time_span.count() << " seconds.";
  std::cout << std::endl;
*/

  //Perform dsl gridsearch3D
  //high_resolution_clock::time_point t3 = high_resolution_clock::now();

  ROS_INFO("Building search graph...");
  grid_.reset(new dsl::Grid3d(length_metric, width_metric, height_metric, 
    occupancy_map.get(),
    1, 1, 1, 1, DSL_OCCUPIED + 1));
  connectivity_.reset(new dsl::Grid3dConnectivity(*grid_));
  gdsl_.reset(new dsl::GridSearch<3>(*grid_, *connectivity_, cost_, true));
  ROS_INFO("Graph built");
  

  //high_resolution_clock::time_point t4 = high_resolution_clock::now();
  //time_span = duration_cast<duration<double>>(t4 - t3);
/*  std::cout << "----LOG: Building DSL search graph. It took me " << time_span.count() << " seconds.";
  std::cout << std::endl;
*/
    //std::ofstream outfile;
    //outfile.open(pwd + "rebuild_time_volumeMapM_seq.log", std::ios::app);
    //outfile<<time_span.count()<<"\t"<<length_metric * width_metric * height_metric<<"\t"<<seq<<std::endl;
    //outfile.close();
  //  return;

//  publishAllPaths();
  publishOccupancyGrid();
  //planAllPaths();
  //publishAllPaths();

  //high_resolution_clock::time_point t5 = high_resolution_clock::now();
  //time_span = duration_cast<duration<double>>(t5 - t1);
/*  std::cout << "----LOG: octomap_data_callback. It took me " << time_span.count() << " seconds.";
  std::cout << std::endl;
*/
}

void DslGrid3D::updateGDSL(std::shared_ptr<octomap::OcTree> tree)
{
  //ROS_INFO("DslGrid3D::updateGDSL(std::shared_ptr<octomap::OcTree> tree)");
  //using namespace std::chrono;
  //high_resolution_clock::time_point t1 = high_resolution_clock::now();
    //high_resolution_clock::time_point re = high_resolution_clock::now();
    for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it)
    {
        Eigen::Vector3d pos(it.getX(), it.getY(), it.getZ());
        pos = posRes(pos);

            
        int n = it.getSize() / res_octomap;
        int i = -n/2;
	it->getOccupancy();
        do{
            int j = -n/2;
            do{
                int k = -n/2;
                do{
	            int idx = ((int) pos(0) + i) + ((int) pos(1) + j) *length_metric + ((int) pos(2) + k) *length_metric*width_metric;
                    Eigen::Vector3d p(pos(0) + i, pos(1) + j, pos(2) + k);

                    if(tree->isNodeOccupied(*it) and occupancy_map[idx] != DSL_OCCUPIED)
                    {
                //Eigen::Vector3d wpos(x, y, z);
                //Eigen::Vector3i wpos(it.getX()/res_octomap, it.getY()/res_octomap, it.getZ()/res_octomap);
                        saftyMarginal(p, true);
                        gdsl_->SetCost(p, DSL_OCCUPIED);
                        occupancy_map[idx] = DSL_OCCUPIED;
               // DslGrid3D::handleSetOccupied(wpos); 
                    }			
                    else if(!tree->isNodeOccupied(*it)  and occupancy_map[idx] >= DSL_UNKNOWN and occupancy_map[idx] < DSL_OCCUPIED)
            //else if((!tree->isNodeOccupied(*it) and occupancy_map[idx] != 1) and occupancy_map[idx] >= DSL_UNKNOWN)
                    {
                //Eigen::Vector3d wpos(x, y, z);
                //gdsl_->SetCost(it.getCoordinate(), 0);
                        gdsl_->SetCost(p, occupancy_map[idx] - DSL_UNKNOWN + 1);
                        occupancy_map[idx] = occupancy_map[idx] - DSL_UNKNOWN + 1;
                        //std::cout<<occupancy_map[idx]<<std::endl;
                //DslGrid3D::handleSetUnoccupied(wpos); 
                    }
                    k++;
                } while (k < n/2);
                j++;
            } while (j < n/2); 
            i++;
        } while ( i < n/2);
    }

        //high_resolution_clock::time_point re2 = high_resolution_clock::now();
        //duration<double> time_span = duration_cast<duration<double>>(re2 - re);
//        std::cout<<"----LOG: update took me "<< time_span.count()<<std::endl;

        //std::ofstream outfile;
        //outfile.open(pwd + "fullUpdate_time_volumeMapM_seq.log", std::ios::app);
        //outfile.open("/home/grammers/temp_log/fullUpdate_time_volumeMapM_seq.log", std::ios::app);
        //outfile<<time_span.count()<<"\t"<<length_metric * width_metric * height_metric<<"\t"<<seq<<std::endl;
        //outfile.close();
        
        //publishOccupancyGrid();
    //setAndPublishPath();
    /*
    if(start_set and goal_set)
    {
        if((int) start_pos(0) == (int) goal_pos(0)
            and (int) start_pos(1) == (int) goal_pos(1)
            and (int) start_pos(2) == (int) goal_pos(2))
        {
            ROS_WARN("Start and goal poses are the some");
            return;
        }
        else if(gdsl_->SetStart(start_pos) and gdsl_->SetGoal(goal_pos))
        {
            //std::cout<<"crach track 1"<<std::endl;
            planAllPaths();
            publishAllPaths();
        }
    }
    */
    return;
    

}

void DslGrid3D::saftyMarginal(Eigen::Vector3d pos, bool update)
{
    //std::cout<<"safty: "<< update << std::endl;
    for(int i = -2; i <= 2; i++){
        for(int j = -2; j <= 2; j++){
            for(int k = -2; k <= 2; k++){
                int x = pos(0) + i;
                int y = pos(1) + j;
                int z = pos(2) + k;
	        //int idx = (int) pos(0) + i + ((int) pos(1) + j)*length_metric + ((int) pos(2) + k)*length_metric*width_metric;
                //why will grid build crach when z >= 0
                if(x >= 0 && x < length_metric && y >= 0 && y < width_metric && z > 0 && z < height_metric){
	            int idx = x + y*length_metric + z*length_metric*width_metric;
                    int sum = i * i + j * j + k * k;
                    if (!sum){
                        continue;
                    }
                    int cost = DSL_UNKNOWN / (sum + 2);
                    if(occupancy_map[idx] < cost){
                        occupancy_map[idx] = cost;
                        if(update){
                            gdsl_->SetCost(pos, cost);
                        }
                    }
                    else if (DSL_UNKNOWN == occupancy_map[idx] or (occupancy_map[idx] > DSL_UNKNOWN and occupancy_map[idx] < DSL_OCCUPIED)){
                        cost += DSL_UNKNOWN;
                        occupancy_map[idx] = cost;
                        if(update){
                            gdsl_->SetCost(pos, cost);
                        }
                        
                    }
                }
            }
        }
    }
}


void DslGrid3D::handleSetStartOdom(const nav_msgs::Odometry msg)
{
  Eigen::Vector3d wpos(msg.pose.pose.position.x , msg.pose.pose.position.y, msg.pose.pose.position.z);
  setStart(wpos);
}


void DslGrid3D::handleSetStart(const geometry_msgs::PointConstPtr& msg)
{
  Eigen::Vector3d wpos(msg->x, msg->y, msg->z);
  setStart(wpos);
}

void DslGrid3D::setStart(Eigen::Vector3d wpos)
{
  //ROS_INFO("Set start pos: %f %f %f", wpos(0), wpos(1), wpos(2));
  start_pos = wpos; 
  setAndPublishPath();
}

void DslGrid3D::handleSetFrontier(const exploration::FrontierConstPtr& msg)
{
  Eigen::Vector3d wpos(msg->point.x, msg->point.y, msg->point.z);
  setGoal(wpos);
}

void DslGrid3D::handleSetGoal(const geometry_msgs::PointConstPtr& msg)
{
  Eigen::Vector3d wpos(msg->x, msg->y, msg->z);
  setGoal(wpos);
}

void DslGrid3D::setGoal(Eigen::Vector3d wpos)
{
    //ROS_INFO("Set goal pos: %f %f %f", wpos(0), wpos(1), wpos(2));
    goal_pos = wpos; 
    setAndPublishPath();
}

void DslGrid3D::setAndPublishPath(){
    if(!grid_built){
        return;
    }
    Eigen::Vector3d grid_start = posRes(start_pos);
    Eigen::Vector3d grid_goal = posRes(goal_pos);

    if((int) grid_start(0) == (int) grid_goal(0)
        and (int) grid_start(1) == (int) grid_goal(1)
        and (int) grid_start(2) == (int) grid_goal(2))
    {
        ROS_WARN("Start and goal poses are the some");
        return;
    }
    if (!gdsl_->SetStart(grid_start))
    {
        ROS_WARN("SetStart faild");
        return;
    }
    if (!gdsl_->SetGoal(grid_goal))
    {
        ROS_WARN("SetGoal faild");
        return;
    }

    planAllPaths();
    publishAllPaths();

}

Eigen::Vector3d DslGrid3D::posRes(Eigen::Vector3d wpos)
{
    for (int i = 0; i < 3; i++)
    {
        wpos(i) = (wpos(i) - pmin(i)) / res_octomap;
    }
    return wpos;
}

void DslGrid3D::handleSetOccupied(const geometry_msgs::PointConstPtr& msg)
{
  //ROS_INFO("DslGrid3D::handleSetOccupied(const geometry_msgs::PointConstPtr& msg)");
  Eigen::Vector3d wpos(msg->x/res_octomap, msg->y/res_octomap, msg->z/res_octomap);
  DslGrid3D::handleSetOccupied(wpos);
}
void DslGrid3D::handleSetOccupied(Eigen::Vector3d wpos)
{
  //ROS_INFO("DslGrid3D::handleSetOccupied(Eigen::Vector3d wpos)");
  //if(!isPosInBounds(wpos))
  //{
  //  ROS_WARN("handleSetOccupied: Position %f %f %f out of bounds!", wpos(0), wpos(1), wpos(2));
  //  return;
  //} 

  //gdsl_->SetCost(ogrid_->positionToDslPosition(wpos), DSL_OCCUPIED);
  gdsl_->SetCost(wpos, DSL_OCCUPIED);

  //std::cout << "Set Occupied pos: " << wpos.transpose() << std::endl;

  //publishOccupancyGrid();
  //planAllPaths();
  //publishAllPaths();
}

void DslGrid3D::handleSetUnoccupied(const geometry_msgs::PointConstPtr& msg)
{
  //ROS_INFO("DslGrid3D::handleSetUnoccupied(const geometry_msgs::PointConstPtr& msg)");
  Eigen::Vector3d wpos(msg->x/res_octomap, msg->y/res_octomap, msg->z/res_octomap);
  handleSetUnoccupied(wpos);
}

void DslGrid3D::handleSetUnoccupied(Eigen::Vector3d wpos)
{
  //ROS_INFO("DslGrid3D::handleSetUnoccupied(Eigen::Vector3d wpos)");
  //if(!isPosInBounds(wpos))
  //{
  //  ROS_WARN("handleSetUnoccupied: Position %f %f %f out of bounds!", wpos(0), wpos(1), wpos(2));
 //   return;
 // } 

  gdsl_->SetCost(wpos, 0);

  //std::cout << "Set Unoccupied pos: " << wpos.transpose() << std::endl;

  //publishOccupancyGrid();
  //planAllPaths();
  //publishAllPaths();
}

//TODO
// fix test sow it test agenst gdsl
bool DslGrid3D::isPosInBounds(const Eigen::Vector3d& pos)
{
  //ROS_INFO("bool DslGrid3D::isPosInBounds(const Eigen::Vector3d& pos)");
  //using namespace std::chrono;
  //high_resolution_clock::time_point t1 = high_resolution_clock::now();
    return true;
  if (pos(0) > pmin(0) && pos(1) > pmin(1) && pos(2) > pmin(2) && pos(0) < pmax(0) 
    && pos(1) < pmax(1) && pos(2) < pmax(2))
    {
        return true;
    } else {
        return false;
    }

  //high_resolution_clock::time_point t2 = high_resolution_clock::now();
  //duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
/*  std::cout << "----LOG: isPosInBounds. It took me " << time_span.count() << " seconds.";
  std::cout << std::endl;
*/
}

void DslGrid3D::planAllPaths()
{
  //ROS_INFO("DslGrid3D::planAllPaths()");
  //using namespace std::chrono;
  //high_resolution_clock::time_point t1 = high_resolution_clock::now();

  gdsl_->Plan(path_);
  //gdsl_->OptPath(path_, optpath_, 1e-3, 1./(10*res_octomap));
  //gdsl_->OptPath(path_, optpath_, 1e-3, 1./(10*cells_per_meter_));
  //gdsl_->SplinePath(path_, splinepath_, /*splinecells_,*/ spline_step_);
  //gdsl_->SplinePath(optpath_, splineoptpath_, /*splineoptcells_,*/ spline_step_);

  //high_resolution_clock::time_point t2 = high_resolution_clock::now();
  //duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
/*  std::cout << "----LOG: planAllPaths. It took me " << time_span.count() << " seconds.";
  std::cout << std::endl;
*/    
    //std::ofstream outfile;
    //outfile.open(pwd + "pathPlaning_time_volumeMapM_seq.log", std::ios::app);
    //outfile<<time_span.count()<<"\t"<<length_metric * width_metric * height_metric<<"\t"<<seq<<std::endl;
    //outfile.close();
    return;
}

void DslGrid3D::publishAllPaths()
{
  //ROS_INFO("DslGrid3D::publishAllPaths()");
  //using namespace std::chrono;
  //high_resolution_clock::time_point t1 = high_resolution_clock::now();

  path_pub_.publish(dslPathToRosMsg(path_));
  optpath_pub_.publish(dslPathToRosMsg(optpath_)); 
  //splinepath_pub_.publish(dslPathToRosMsg(splinepath_)); 
  //splineoptpath_pub_.publish(dslPathToRosMsg(splineoptpath_)); 

  //high_resolution_clock::time_point t2 = high_resolution_clock::now();
  //duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
/*  std::cout << "----LOG: publishAllPaths. It took me " << time_span.count() << " seconds.";
  std::cout << std::endl;
*/
}

nav_msgs::Path DslGrid3D::dslPathToRosMsg(const dsl::GridPath<3> &dsl_path)
{
  //ROS_INFO("nav_msgs::Path DslGrid3D::dslPathToRosMsg(const dsl::GridPath<3> &dsl_path)");
  //using namespace std::chrono;
  //high_resolution_clock::time_point t1 = high_resolution_clock::now();

  std::vector<Eigen::Vector3d> path;
  for(int i = 0; i < dsl_path.cells.size(); i++)
  {
    path.push_back(dsl_path.cells[i].c * res_octomap);
  }

  //high_resolution_clock::time_point t2 = high_resolution_clock::now();
  //duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
/*  std::cout << "----LOG: dslPathToRosMsg1. It took me " << time_span.count() << " seconds.";
  std::cout << std::endl;
*/
  return dslPathToRosMsg(path);
}

nav_msgs::Path DslGrid3D::dslPathToRosMsg(const std::vector<Eigen::Vector3d> &dsl_path)
{
  //ROS_INFO("nav_msgs::Path DslGrid3D::dslPathToRosMsg(const std::vector<Eigen::Vector3d> &dsl_path)");
  //using namespace std::chrono;
  //high_resolution_clock::time_point t1 = high_resolution_clock::now();

  nav_msgs::Path msg;  
  
  msg.header.frame_id = odom_frame_id_; ///world /pixy/velodyne
  msg.poses.resize(dsl_path.size());
  for(int i = 0; i < dsl_path.size(); i++)
  {
    msg.poses[i].pose.position.x = dsl_path[i][0] + pmin(0) + res_octomap / 2 - 0.25;
    msg.poses[i].pose.position.y = dsl_path[i][1] + pmin(1) + res_octomap / 2 - 0.25;
    msg.poses[i].pose.position.z = dsl_path[i][2] + pmin(2) + res_octomap / 2 - 0.25;
  }

  //high_resolution_clock::time_point t2 = high_resolution_clock::now();
  //duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
/*  std::cout << "----LOG: dslPathToRosMsg2. It took me " << time_span.count() << " seconds.";
  std::cout << std::endl;
*/
  return msg; 
}

void DslGrid3D::publishOccupancyGrid()
{
  //ROS_INFO("DslGrid3D::publishOccupancyGrid()");
  //using namespace std::chrono;
  //high_resolution_clock::time_point t1 = high_resolution_clock::now();

  visualization_msgs::Marker occmap_viz;

  std::vector<geometry_msgs::Point> marker_pos;
  std::vector<geometry_msgs::Point> risk_pos;
     
  for(double x = 0; x < length_metric; x++)
  {
    for(double y = 0; y < width_metric; y++)
    {
      for(double z = 0; z < height_metric; z++)
      {
        //Eigen::Vector3d pos((x - pmin(0)) / res_octomap, (y - pmin(1)) / res_octomap, (z - pmin(2)) / res_octomap);
        //Eigen::Vector3d pos(x - pmin(0), y - pmin(1), z - pmin(2));
        Eigen::Vector3d pos(x, y, z);
        int idx = x + y*length + z*length*width;
        //assert(!(idx >= length*width*height || idx < 0));
        //if(!isPosInBounds(pos)){
        //    std::cout<<"out of bounds"<<std::endl;
        //    continue;
        //}

        //std::cout<<"check 01: "<<gdsl_->GetCost(pos)<<std::endl;
        if(gdsl_->GetCost(pos) == DSL_OCCUPIED)
        //if(gdsl_->GetCost(pos) == DSL_UNKNOWN)
        //if(gdsl_->GetCost(pos) == 1)
        //if(gdsl_->GetCost(pos) > 1 and gdsl_->GetCost(pos) < DSL_UNKNOWN)
        //if(gdsl_->GetCost(pos) < DSL_OCCUPIED and gdsl_->GetCost(pos) > DSL_UNKNOWN)
        {
          //std::cout << "pt occupied: " << x << " " << y << " " << z << std::endl;
          geometry_msgs::Point pt;
          pt.x = (x + pmin(0)) * res_octomap;
          pt.y = (y + pmin(1)) * res_octomap;
          pt.z = (z + pmin(2)) * res_octomap;
          marker_pos.push_back(pt);
        }  
        else if (2 < gdsl_->GetCost(pos) and gdsl_->GetCost(pos) < DSL_UNKNOWN)
        {
          geometry_msgs::Point pt;
          pt.x = (x + pmin(0)) * res_octomap;
          pt.y = (y + pmin(1)) * res_octomap;
          pt.z = (z + pmin(2)) * res_octomap;
          risk_pos.push_back(pt);
            
        }
      }  
    }
  }

  occmap_viz.header.frame_id = odom_frame_id_; ///world /pixy/velodyne
  occmap_viz.header.stamp = ros::Time();
  occmap_viz.ns = "dsl_grid3d";
  occmap_viz.id = 1;
  occmap_viz.type = visualization_msgs::Marker::CUBE_LIST;
  occmap_viz.action = visualization_msgs::Marker::ADD;
  occmap_viz.pose.position.x = 0.5 * res_octomap + pmin(0) * res_octomap; 
  occmap_viz.pose.position.y = 0.5 * res_octomap + pmin(1) * res_octomap;
  occmap_viz.pose.position.z = 0.5 * res_octomap + pmin(2) * res_octomap;
  occmap_viz.pose.orientation.x = 0.0;
  occmap_viz.pose.orientation.y = 0.0;
  occmap_viz.pose.orientation.z = 0.0;
  occmap_viz.pose.orientation.w = 1.0;
  occmap_viz.scale.x = 1.0 * res_octomap;
  occmap_viz.scale.y = 1.0 * res_octomap;
  occmap_viz.scale.z = 1.0 * res_octomap;
  occmap_viz.color.a = 0.5;
  occmap_viz.color.r = 1.0;
  occmap_viz.color.g = 0.0;
  occmap_viz.color.b = 0.0;
  occmap_viz.points = marker_pos;
/*
  occmap_viz.header.frame_id = odom_frame_id_; ///world /pixy/velodyne
  occmap_viz.header.stamp = ros::Time();
  occmap_viz.ns = "dsl_grid3d";
  occmap_viz.id = 1;
  occmap_viz.type = visualization_msgs::Marker::CUBE_LIST;
  occmap_viz.action = visualization_msgs::Marker::ADD;
  occmap_viz.pose.position.x = 0.5 * res_octomap + pmin(0) * res_octomap; 
  occmap_viz.pose.position.y = 0.5 * res_octomap + pmin(1) * res_octomap;
  occmap_viz.pose.position.z = 0.5 * res_octomap + pmin(2) * res_octomap;
  occmap_viz.pose.orientation.x = 0.0;
  occmap_viz.pose.orientation.y = 0.0;
  occmap_viz.pose.orientation.z = 0.0;
  occmap_viz.pose.orientation.w = 1.0;
  occmap_viz.scale.x = 1.0 * res_octomap;
  occmap_viz.scale.y = 1.0 * res_octomap;
  occmap_viz.scale.z = 1.0 * res_octomap;
  occmap_viz.color.a = 0.5;
  occmap_viz.color.r = 0.0;
  occmap_viz.color.g = 1.0;
  occmap_viz.color.b = 0.0;
  occmap_viz.points = risk_pos;
*/
  occ_map_viz_pub_.publish(occmap_viz);

  //high_resolution_clock::time_point t2 = high_resolution_clock::now();
  //duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
/*  std::cout << "----LOG: publishOccupancyGrid. It took me " << time_span.count() << " seconds.";
  std::cout << std::endl;
*/
}

} // namespace
