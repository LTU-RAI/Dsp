#include "dsl_gridsearch/dsl_grid3d.h"

#include <visualization_msgs/Marker.h>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>

#include <chrono>

namespace dsl_gridsearch
{

DslGrid3D::DslGrid3D(ros::NodeHandle nh, ros::NodeHandle nh_private) :
  nh_(nh),
  nh_private_(nh_private)
{
  double grid_xmin, grid_ymin, grid_zmin;

  if (!nh_private_.getParam ("cells_per_meter", cells_per_meter_))
    cells_per_meter_ = 1.;
  if (!nh_private_.getParam ("use_gazebo_odom", use_gazebo_odom_))
    use_gazebo_odom_ = false;
/*  if (!nh_private_.getParam ("spline_step_", spline_step_))
    spline_step_ = .1;*/

  occ_map_viz_pub_ = nh_.advertise<visualization_msgs::Marker>( "/dsl_grid3d/occupancy_map",  0);
  path_pub_ = nh_.advertise<nav_msgs::Path>( "/dsl_grid3d/path",  0);
  optpath_pub_ = nh_.advertise<nav_msgs::Path>( "/dsl_grid3d/optpath",  0);
/*  splinepath_pub_ = nh_.advertise<nav_msgs::Path>( "/dsl_grid3d/splinepath",  0);
  splineoptpath_pub_ = nh_.advertise<nav_msgs::Path>( "/dsl_grid3d/splineoptpath",  0);*/

  if(use_gazebo_odom_)
  {
    set_start_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/pixy/truth/NWU", 10, 
      &DslGrid3D::handleSetStartOdom, this, ros::TransportHints().tcpNoDelay());
  }
  else
  {
    set_start_sub_ = nh_.subscribe<geometry_msgs::Point>("/dsl_grid3d/set_start", 10, 
      &DslGrid3D::handleSetStart, this, ros::TransportHints().tcpNoDelay());
  }

  set_goal_sub_ = nh_.subscribe<geometry_msgs::Point>("/dsl_grid3d/set_goal", 10,
    &DslGrid3D::handleSetGoal, this, ros::TransportHints().tcpNoDelay());
  set_occupied_sub_ = nh_.subscribe<geometry_msgs::Point>("/dsl_grid3d/set_occupied", 10, 
    &DslGrid3D::handleSetOccupied, this, ros::TransportHints().tcpNoDelay());
  set_unoccupied_sub_ = nh_.subscribe<geometry_msgs::Point>("/dsl_grid3d/set_unoccupied", 10, 
    &DslGrid3D::handleSetUnoccupied, this, ros::TransportHints().tcpNoDelay());
  get_octomap_sub_ = nh_.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10,
    &DslGrid3D::octomap_data_callback, this, ros::TransportHints().tcpNoDelay());

  //timer = nh_private_.createTimer(ros::Duration(0.1), &DslGrid3D::spin, this);
  //ROS_INFO("Spinner started");
}

void DslGrid3D::octomap_data_callback(const octomap_msgs::OctomapConstPtr& msg)
{	
  using namespace std::chrono;
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

    octomap::AbstractOcTree* treeptr = octomap_msgs::msgToMap(*msg);
    // This can be cast to other types. An octree seems to be appropriate for publishing
    octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(treeptr);

	std::cout << "resolution: " << tree->getResolution() << std::endl;
	std::cout << "type: " << tree->getTreeType() << std::endl;
	std::cout << "size: " << tree->size() << std::endl;

    res_octomap = tree->getResolution();
	double length,width,height, xmin, ymin, zmin, xmax, ymax, zmax, cells_per_meter;
    tree->getMetricSize(length,width,height);
    tree->getMetricMin(xmin, ymin, zmin);
    tree->getMetricMax(xmax, ymax, zmax);

    std::cout << "Metric size. length: " << length/res_octomap << " width: " << width/res_octomap << " height: " << height/res_octomap << std::endl;
	int length_metric = length/res_octomap;
	int width_metric = width/res_octomap;
	int height_metric = height/res_octomap;

    const double occupied_val = DSL_OCCUPIED;
    int count = 0;
	double *occupancy_map = new double[length_metric*width_metric*height_metric];

   for(int i = 0; i < length_metric*width_metric*height_metric; i++)
   {
     occupancy_map[i] = 0; //occupied_val;
   }

   for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it)
    {
        count++;
      	int x = it.getX()/res_octomap - xmin/res_octomap;
		int y = it.getY()/res_octomap - ymin/res_octomap;
		int z = it.getZ()/res_octomap - zmin/res_octomap;
//	    std::cout << "Metric. x: " << x << " y: " << y << " z: " << z << std::endl;
		it->getOccupancy();
	    if(tree->isNodeOccupied(*it)) {
//		        std::cout << " Occupied" << std::endl;
			int idx = x + y*length_metric + z*length_metric*width_metric;
			assert(!(idx >= length_metric*width_metric*height_metric || idx < 0));
//				std::cout << "idx: " << idx << std::endl;
			occupancy_map[idx] = occupied_val;
		}			
    }
	std::cout << "count: " << count << std::endl;
	ogrid_ = new OccupancyGrid(occupancy_map, length_metric, width_metric, height_metric, 
      Eigen::Vector3d(xmin/res_octomap, ymin/res_octomap, zmin/res_octomap), 
      Eigen::Vector3d(xmax/res_octomap, ymax/res_octomap, zmax/res_octomap), 1);

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
	std::cout << "Grid Bounds: " << ogrid_->getPmin().transpose() << " and " 
    << ogrid_->getPmax().transpose() << std:: endl;

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  std::cout << "----LOG: Build OccupancyGrid. It took me " << time_span.count() << " seconds.";
  std::cout << std::endl;

  //Perform dsl gridsearch3D
  high_resolution_clock::time_point t3 = high_resolution_clock::now();

  ROS_INFO("Building search graph...");
/*  Grid3d(int length, int width, int height, double *map, 
           double sx, double sy, double sz, double costScale,
           double maxCost = 1);*/

  grid_ = new dsl::Grid3d(ogrid_->getLength(), ogrid_->getWidth(), ogrid_->getHeight(), 
    ogrid_->getOccupancyMap(), 
    1/cells_per_meter_, 1/cells_per_meter_, 1/cells_per_meter_, 1, 1000);
  std::cout << "ogrid_->getOccupancyMap(): " << ogrid_->getOccupancyMap() << std::endl;
  connectivity_ = new dsl::Grid3dConnectivity(*grid_);
  gdsl_ = new dsl::GridSearch<3>(*grid_, *connectivity_, cost_, true);
//  gdsl_->SetStart(Eigen::Vector3d(0, 0, 0));
//  gdsl_->SetGoal(Eigen::Vector3d(0, 0, 0));
  ROS_INFO("Graph built");
  

  high_resolution_clock::time_point t4 = high_resolution_clock::now();
  time_span = duration_cast<duration<double>>(t4 - t3);
  std::cout << "----LOG: Building DSL search graph. It took me " << time_span.count() << " seconds.";
  std::cout << std::endl;

//  publishAllPaths();
  publishOccupancyGrid();

  high_resolution_clock::time_point t5 = high_resolution_clock::now();
  time_span = duration_cast<duration<double>>(t5 - t1);
  std::cout << "----LOG: octomap_data_callback. It took me " << time_span.count() << " seconds.";
  std::cout << std::endl;
}

void DslGrid3D::spin(const ros::TimerEvent& e)
{
}

void DslGrid3D::handleSetStartOdom(const nav_msgs::Odometry msg)
{
  Eigen::Vector3d wpos(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
  std::cout << "wpos: " << wpos << std::endl;
  if(!isPosInBounds(wpos))
  {
    ROS_WARN("handleSetStartOdom: Position %f %f %f out of bounds!", wpos(0), wpos(1), wpos(2));
    return;
  }
 
  ROS_INFO("Set start pos: %f %f %f", wpos(0), wpos(1), wpos(2));
  Eigen::Vector3d pos = ogrid_->positionToDslPosition(wpos);
  gdsl_->SetStart(pos); 
//  gdsl_->SetStart(wpos);

//  planAllPaths();
//  publishAllPaths();
}


void DslGrid3D::handleSetStart(const geometry_msgs::PointConstPtr& msg)
{
  using namespace std::chrono;
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  Eigen::Vector3d wpos(msg->x/res_octomap, msg->y/res_octomap, msg->z/res_octomap);

  if(!isPosInBounds(wpos))
  {
    ROS_WARN("handleSetStart: Position %f %f %f out of bounds!", wpos(0), wpos(1), wpos(2));
    return;
  }
 
  ROS_INFO("Set start pos: %f %f %f", wpos(0), wpos(1), wpos(2));
  Eigen::Vector3d pos = ogrid_->positionToDslPosition(wpos);
  gdsl_->SetStart(pos); 
//  gdsl_->SetStart(wpos);

//  planAllPaths();
//  publishAllPaths();

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  std::cout << "----LOG: handleSetStart. It took me " << time_span.count() << " seconds.";
  std::cout << std::endl;
}

void DslGrid3D::handleSetGoal(const geometry_msgs::PointConstPtr& msg)
{
  using namespace std::chrono;
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  Eigen::Vector3d wpos(msg->x/res_octomap, msg->y/res_octomap, msg->z/res_octomap);

  if(!isPosInBounds(wpos))
  {
    ROS_WARN("handleSetGoal: Position %f %f %f out of bounds!", wpos(0), wpos(1), wpos(2));
    return;
  } 

  ROS_INFO("Set goal pos: %f %f %f", wpos(0), wpos(1), wpos(2));
  Eigen::Vector3d pos = ogrid_->positionToDslPosition(wpos);
  ROS_INFO("Set dsl goal pos: %f %f %f", pos(0), pos(1), pos(2));
  gdsl_->SetGoal(pos);
//  gdsl_->SetGoal(wpos);

  planAllPaths();
  publishAllPaths();

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  std::cout << "----LOG: handleSetGoal. It took me " << time_span.count() << " seconds.";
  std::cout << std::endl;
}
void DslGrid3D::handleSetOccupied(const geometry_msgs::PointConstPtr& msg)
{
  Eigen::Vector3d wpos(msg->x/res_octomap, msg->y/res_octomap, msg->z/res_octomap);

  if(!isPosInBounds(wpos))
  {
    ROS_WARN("handleSetOccupied: Position %f %f %f out of bounds!", wpos(0), wpos(1), wpos(2));
    return;
  } 

  Eigen::Vector3i gpos = ogrid_->positionToGrid(wpos);
  ogrid_->setOccupied(wpos, true);
  gdsl_->SetCost(ogrid_->positionToDslPosition(wpos), DSL_OCCUPIED);
//  gdsl_->SetCost(wpos, DSL_OCCUPIED);

  std::cout << "Set Occupied pos: " << wpos.transpose() << std::endl;

  publishOccupancyGrid();
  planAllPaths();
  publishAllPaths();
}
void DslGrid3D::handleSetUnoccupied(const geometry_msgs::PointConstPtr& msg)
{
  Eigen::Vector3d wpos(msg->x/res_octomap, msg->y/res_octomap, msg->z/res_octomap);

  if(!isPosInBounds(wpos))
  {
    ROS_WARN("handleSetUnoccupied: Position %f %f %f out of bounds!", wpos(0), wpos(1), wpos(2));
    return;
  } 

  Eigen::Vector3i gpos = ogrid_->positionToGrid(wpos);
  ogrid_->setOccupied(wpos, false);
  gdsl_->SetCost(ogrid_->positionToDslPosition(wpos), 0);
//  gdsl_->SetCost(wpos, 0);

  std::cout << "Set Unoccupied pos: " << wpos.transpose() << std::endl;

  publishOccupancyGrid();
  planAllPaths();
  publishAllPaths();
}

bool DslGrid3D::isPosInBounds(const Eigen::Vector3d& pos)
{
  using namespace std::chrono;
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  Eigen::Vector3d pmin = ogrid_->getPmin();
  Eigen::Vector3d pmax = ogrid_->getPmax();
  return (pos(0) >= pmin(0) && pos(1) >= pmin(1) && pos(2) >= pmin(2) && pos(0) <= pmax(0) 
    && pos(1) <= pmax(1) && pos(2) <= pmax(2));

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  std::cout << "----LOG: isPosInBounds. It took me " << time_span.count() << " seconds.";
  std::cout << std::endl;
}

void DslGrid3D::planAllPaths()
{
  using namespace std::chrono;
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  gdsl_->Plan(path_);
  gdsl_->OptPath(path_, optpath_, 1e-3, 1./(10*cells_per_meter_));
//  gdsl_->SplinePath(path_, splinepath_, /*splinecells_,*/ spline_step_);
//  gdsl_->SplinePath(optpath_, splineoptpath_, /*splineoptcells_,*/ spline_step_);

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  std::cout << "----LOG: planAllPaths. It took me " << time_span.count() << " seconds.";
  std::cout << std::endl;
}

void DslGrid3D::publishAllPaths()
{
  using namespace std::chrono;
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  path_pub_.publish(dslPathToRosMsg(path_));
  optpath_pub_.publish(dslPathToRosMsg(optpath_)); 
/*  splinepath_pub_.publish(dslPathToRosMsg(splinepath_)); 
  splineoptpath_pub_.publish(dslPathToRosMsg(splineoptpath_)); */

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  std::cout << "----LOG: publishAllPaths. It took me " << time_span.count() << " seconds.";
  std::cout << std::endl;
}

nav_msgs::Path DslGrid3D::dslPathToRosMsg(const dsl::GridPath<3> &dsl_path)
{
  using namespace std::chrono;
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  std::vector<Eigen::Vector3d> path;
  for(int i = 0; i < dsl_path.cells.size(); i++)
  {
    path.push_back(dsl_path.cells[i].c * res_octomap);
  }


  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  std::cout << "----LOG: dslPathToRosMsg1. It took me " << time_span.count() << " seconds.";
  std::cout << std::endl;

  return dslPathToRosMsg(path);
}
nav_msgs::Path DslGrid3D::dslPathToRosMsg(const std::vector<Eigen::Vector3d> &dsl_path)
{
  using namespace std::chrono;
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  nav_msgs::Path msg;  
  
  msg.header.frame_id = "odom"; ///world /pixy/velodyne
  msg.poses.resize(dsl_path.size());
  double xmin = ogrid_->getPmin()(0) * res_octomap;
  double ymin = ogrid_->getPmin()(1) * res_octomap;
  double zmin = ogrid_->getPmin()(2) * res_octomap;
  for(int i = 0; i < dsl_path.size(); i++)
  {
    msg.poses[i].pose.position.x = dsl_path[i][0] + xmin;//0.5;
    msg.poses[i].pose.position.y = dsl_path[i][1] + ymin;//0.5;
    msg.poses[i].pose.position.z = dsl_path[i][2] + zmin;//0.5;
  }

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  std::cout << "----LOG: dslPathToRosMsg2. It took me " << time_span.count() << " seconds.";
  std::cout << std::endl;

  return msg; 
}

void DslGrid3D::publishOccupancyGrid()
{
  using namespace std::chrono;
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  visualization_msgs::Marker occmap_viz;

  std::vector<geometry_msgs::Point> marker_pos;
  int length = ogrid_->getLength();
  int width = ogrid_->getWidth();
  int height = ogrid_->getHeight();
  for(int x = 0; x < length; x++)
  {
    for(int y = 0; y < width; y++)
    {
      for(int z = 0; z < height; z++)
      {
        int idx = x + y*length + z*length*width;
        assert(!(idx >= length*width*height || idx < 0));
        if(ogrid_->getOccupancyMap()[idx] == DSL_OCCUPIED)
        {
          //std::cout << "pt occupied: " << x << " " << y << " " << z << std::endl;
          geometry_msgs::Point pt;
          pt.x = x/cells_per_meter_ * res_octomap;
          pt.y = y/cells_per_meter_ * res_octomap;
          pt.z = z/cells_per_meter_ * res_octomap;
          marker_pos.push_back(pt);
        }
      }  
    }
  }

  occmap_viz.header.frame_id = "odom"; ///world /pixy/velodyne
  occmap_viz.header.stamp = ros::Time();
  occmap_viz.ns = "dsl_grid3d";
  occmap_viz.id = 1;
  occmap_viz.type = visualization_msgs::Marker::CUBE_LIST;
  occmap_viz.action = visualization_msgs::Marker::ADD;
  occmap_viz.pose.position.x = 0.0/(2.*cells_per_meter_) + ogrid_->getPmin()(0) * res_octomap + res_octomap/2;
  occmap_viz.pose.position.y = 0.0/(2.*cells_per_meter_) + ogrid_->getPmin()(1) * res_octomap + res_octomap/2;
  occmap_viz.pose.position.z = 0.0/(2.*cells_per_meter_) + ogrid_->getPmin()(2) * res_octomap + res_octomap/2;
  occmap_viz.pose.orientation.x = 0.0;
  occmap_viz.pose.orientation.y = 0.0;
  occmap_viz.pose.orientation.z = 0.0;
  occmap_viz.pose.orientation.w = 1.0;
  occmap_viz.scale.x = 1.0/cells_per_meter_ * res_octomap;
  occmap_viz.scale.y = 1.0/cells_per_meter_ * res_octomap;
  occmap_viz.scale.z = 1.0/cells_per_meter_ * res_octomap;
  occmap_viz.color.a = 0.5;
  occmap_viz.color.r = 1.0;
  occmap_viz.color.g = 0.0;
  occmap_viz.color.b = 0.0;
  occmap_viz.points = marker_pos;

  occ_map_viz_pub_.publish(occmap_viz);

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  std::cout << "----LOG: publishOccupancyGrid. It took me " << time_span.count() << " seconds.";
  std::cout << std::endl;
}

} // namespace
