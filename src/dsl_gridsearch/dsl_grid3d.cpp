#include "dsl_gridsearch/dsl_grid3d.h"
#include "dsl_gridsearch/mesh_utility.h"

#include <visualization_msgs/Marker.h>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>

namespace dsl_gridsearch
{

DslGrid3D::DslGrid3D(ros::NodeHandle nh, ros::NodeHandle nh_private) :
  nh_(nh),
  nh_private_(nh_private),
  mesh_filename_("")
{
  double grid_xmin, grid_ymin, grid_zmin;

  if (!nh_private_.getParam ("mesh_filename", mesh_filename_))
    mesh_filename_ = "";
  if (!nh_private_.getParam ("cells_per_meter", cells_per_meter_))
    cells_per_meter_ = 1.;
  if (!nh_private_.getParam ("spline_step_", spline_step_))
    spline_step_ = .1;
  if (!nh_private_.getParam ("use_textured_mesh", use_textured_mesh_))
    use_textured_mesh_ = false;
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

  occ_map_viz_pub_ = nh_.advertise<visualization_msgs::Marker>( "/dsl_grid3d/occupancy_map",  0);
  path_pub_ = nh_.advertise<nav_msgs::Path>( "/dsl_grid3d/path",  0);
  optpath_pub_ = nh_.advertise<nav_msgs::Path>( "/dsl_grid3d/optpath",  0);
  splinepath_pub_ = nh_.advertise<nav_msgs::Path>( "/dsl_grid3d/splinepath",  0);
  splineoptpath_pub_ = nh_.advertise<nav_msgs::Path>( "/dsl_grid3d/splineoptpath",  0);

  set_start_sub_ = nh_.subscribe<geometry_msgs::Point>("/dsl_grid3d/set_start", 10, 
    &DslGrid3D::handleSetStart, this, ros::TransportHints().tcpNoDelay());
  set_goal_sub_ = nh_.subscribe<geometry_msgs::Point>("/dsl_grid3d/set_goal", 10,
    &DslGrid3D::handleSetGoal, this, ros::TransportHints().tcpNoDelay());
  set_occupied_sub_ = nh_.subscribe<geometry_msgs::Point>("/dsl_grid3d/set_occupied", 10, 
    &DslGrid3D::handleSetOccupied, this, ros::TransportHints().tcpNoDelay());
  set_unoccupied_sub_ = nh_.subscribe<geometry_msgs::Point>("/dsl_grid3d/set_unoccupied", 10, 
    &DslGrid3D::handleSetUnoccupied, this, ros::TransportHints().tcpNoDelay());

  get_octomap_sub_ = nh_.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10,
    &DslGrid3D::octomap_data_callback, this, ros::TransportHints().tcpNoDelay());

  timer = nh_private_.createTimer(ros::Duration(5.0), &DslGrid3D::spin, this);
  //ROS_INFO("Spinner started");
  
  sPose << 0.0, 0.0, 0.0;
  gPose << 1.0, 1.0, 1.0;
}

void DslGrid3D::octomap_data_callback(const octomap_msgs::OctomapConstPtr& msg)
{	

    // This can be cast to other types. An octree seems to be appropriate for publishing
    std::unique_ptr<octomap::OcTree> tree(dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg)));
	std::cout << "resolution: " << tree->getResolution() << std::endl;
	std::cout << "type: " << tree->getTreeType() << std::endl;
	std::cout << "size: " << tree->size() << std::endl;

    res = tree->getResolution();
    double length,width,height, xmin, ymin, zmin, xmax, ymax, zmax, cells_per_meter;
    tree->getMetricSize(length,width,height);
    tree->getMetricMin(xmin, ymin, zmin);
    tree->getMetricMax(xmax, ymax, zmax);

    std::cout << "Metric size. length: " << length/res << " width: " << width/res << " height: " << height/res << std::endl;
    int length_metric = length/res;
    int width_metric = width/res;
    int height_metric = height/res;

    int count = 0;
    std::shared_ptr<double[]> occupancy_map(new double[length_metric*width_metric*height_metric]);
        

   for(int i = 0; i < length_metric * width_metric*height_metric; i++)
   {
     occupancy_map[i] = 0; 
   }

   for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it)
    {
        count++;
      	int x = it.getX()/res - xmin/res;
		int y = it.getY()/res - ymin/res;
		int z = it.getZ()/res - zmin/res;
//	    std::cout << "Metric. x: " << x << " y: " << y << " z: " << z << std::endl;
		it->getOccupancy();
	    if(tree->isNodeOccupied(*it)) {
//		        std::cout << " Occupied" << std::endl;
			int idx = x + y*length_metric + z*length_metric*width_metric;
			assert(!(idx >= length_metric*width_metric*height_metric || idx < 0));
//				std::cout << "idx: " << idx << std::endl;
			occupancy_map[idx] = DSL_OCCUPIED;
		}			
    }
	std::cout << "count: " << count << std::endl;
	ogrid_.reset(new OccupancyGrid(occupancy_map, length_metric, width_metric, height_metric, Eigen::Vector3d(xmin/res, ymin/res, zmin/res), Eigen::Vector3d(xmax/res, ymax/res, zmax/res), 1));



//-----------------------------------------
	std::cout << "Grid Bounds: " << ogrid_->getPmin().transpose() << " and " 
    << ogrid_->getPmax().transpose() << std:: endl;

//  grid_.reset(new dsl::Grid3d(ogrid_->getLength(), ogrid_->getWidth(), ogrid_->getHeight(), 
//    ogrid_->getOccupancyMap().get(), 
//    1/cells_per_meter_, 1/cells_per_meter_, 1/cells_per_meter_, 1, 1000));


  publishOccupancyGrid();

//-------------------------------------------------------

}

void DslGrid3D::spin(const ros::TimerEvent& e)
{
  //Perform dsl gridsearch3D
  ROS_INFO("Building search graph...");
  std::shared_ptr<dsl::Grid3d> grid_(new dsl::Grid3d(ogrid_->getLength(), ogrid_->getWidth(), ogrid_->getHeight(), 
    ogrid_->getOccupancyMap().get(), 
    1/cells_per_meter_, 1/cells_per_meter_, 1/cells_per_meter_, 1, 1000));
//  std::cout << "ogrid_->getOccupancyMap(): " << ogrid_->getOccupancyMap() << std::endl;
  gdsl_.reset(new dsl::GridSearch<3>(*grid_, dsl::Grid3dConnectivity(*grid_), cost_, true));
  ROS_INFO("Set start pos: %f %f %f", gPose(0), gPose(1), gPose(2));
  gdsl_->SetStart(ogrid_->positionToDslPosition(sPose));
  gdsl_->SetGoal(ogrid_->positionToDslPosition(gPose));
  ROS_INFO("Graph built");

  planAllPaths();
  publishAllPaths();
}

void DslGrid3D::handleSetStart(const geometry_msgs::PointConstPtr& msg)
{
//  Eigen::Vector3d wpos(msg->x/res_given, msg->y/res_given, msg->z/res_given);
  sPose(0) = msg->x/res;
  sPose(1) =  msg->y/res;
  sPose(2) =  msg->z/res;

  if(!isPosInBounds(sPose))
  {
    ROS_WARN("handleSetStart: Position %f %f %f out of bounds!", sPose(0), sPose(1), sPose(2));
    return;
  }
 
  ROS_INFO("Set start pos: %f %f %f", sPose(0), sPose(1), sPose(2));
  Eigen::Vector3d pos = ogrid_->positionToDslPosition(sPose);
  gdsl_->SetStart(pos);

  //planAllPaths();
  //publishAllPaths();
}

void DslGrid3D::handleSetGoal(const geometry_msgs::PointConstPtr& msg)
{
//  Eigen::Vector3d wpos(msg->x/res_given, msg->y/res_given, msg->z/res_given);
  gPose(0) = msg->x/res;
  gPose(1) =  msg->y/res;
  gPose(2) =  msg->z/res;

  if(!isPosInBounds(gPose))
  {
    ROS_WARN("handleSetGoal: Position %f %f %f out of bounds!", gPose(0), gPose(1), gPose(2));
    return;
  } 

  ROS_INFO("Set goal pos: %f %f %f", gPose(0), gPose(1), gPose(2));
  Eigen::Vector3d pos = ogrid_->positionToDslPosition(gPose);
  ROS_INFO("Set dsl goal pos: %f %f %f", pos(0), pos(1), pos(2));
  gdsl_->SetGoal(pos);

  //planAllPaths();
  //publishAllPaths();
}
void DslGrid3D::handleSetOccupied(const geometry_msgs::PointConstPtr& msg)
{
  Eigen::Vector3d wpos(msg->x, msg->y, msg->z);

  if(!isPosInBounds(wpos))
  {
    ROS_WARN("handleSetOccupied: Position %f %f %f out of bounds!", wpos(0), wpos(1), wpos(2));
    return;
  } 

  Eigen::Vector3i gpos = ogrid_->positionToGrid(wpos);
  ogrid_->setOccupied(wpos, true);
  gdsl_->SetCost(ogrid_->positionToDslPosition(wpos), DSL_OCCUPIED);

  std::cout << "Set Occupied pos: " << wpos.transpose() << std::endl;

  publishOccupancyGrid();
  planAllPaths();
  publishAllPaths();
}
void DslGrid3D::handleSetUnoccupied(const geometry_msgs::PointConstPtr& msg)
{
  Eigen::Vector3d wpos(msg->x, msg->y, msg->z);

  if(!isPosInBounds(wpos))
  {
    ROS_WARN("handleSetUnoccupied: Position %f %f %f out of bounds!", wpos(0), wpos(1), wpos(2));
    return;
  } 

  Eigen::Vector3i gpos = ogrid_->positionToGrid(wpos);
  ogrid_->setOccupied(wpos, false);
  gdsl_->SetCost(ogrid_->positionToDslPosition(wpos), 0);

  std::cout << "Set Unoccupied pos: " << wpos.transpose() << std::endl;

  publishOccupancyGrid();
  planAllPaths();
  publishAllPaths();
}

bool DslGrid3D::isPosInBounds(const Eigen::Vector3d& pos)
{
  Eigen::Vector3d pmin = ogrid_->getPmin();
  Eigen::Vector3d pmax = ogrid_->getPmax();
  return (pos(0) >= pmin(0) && pos(1) >= pmin(1) && pos(2) >= pmin(2) && pos(0) <= pmax(0) 
    && pos(1) <= pmax(1) && pos(2) <= pmax(2));
}

void DslGrid3D::planAllPaths()
{
  gdsl_->Plan(path_);
  gdsl_->OptPath(path_, optpath_, 1e-3, 1./(10*cells_per_meter_));
  gdsl_->SplinePath(path_, splinepath_, /*splinecells_,*/ spline_step_);
  gdsl_->SplinePath(optpath_, splineoptpath_, /*splineoptcells_,*/ spline_step_);
}

void DslGrid3D::publishAllPaths()
{
  path_pub_.publish(dslPathToRosMsg(path_)); 
  optpath_pub_.publish(dslPathToRosMsg(optpath_)); 
  splinepath_pub_.publish(dslPathToRosMsg(splinepath_)); 
  splineoptpath_pub_.publish(dslPathToRosMsg(splineoptpath_)); 
}

nav_msgs::Path DslGrid3D::dslPathToRosMsg(const dsl::GridPath<3> &dsl_path)
{
  std::vector<Eigen::Vector3d>  path;
  for(int i = 0; i < dsl_path.cells.size(); i++)
  {
    path.push_back(dsl_path.cells[i].c);
  }
  return dslPathToRosMsg(path);
}
nav_msgs::Path DslGrid3D::dslPathToRosMsg(const std::vector<Eigen::Vector3d> &dsl_path)
{
  nav_msgs::Path msg;  
  
  msg.header.frame_id = "/world";
  msg.poses.resize(dsl_path.size());
  double xmin = ogrid_->getPmin()(0);
  double ymin = ogrid_->getPmin()(1);
  double zmin = ogrid_->getPmin()(2);
  for(int i = 0; i < dsl_path.size(); i++)
  {
    msg.poses[i].pose.position.x = (dsl_path[i][0] + xmin) * res;
    msg.poses[i].pose.position.y = (dsl_path[i][1] + ymin) * res;
    msg.poses[i].pose.position.z = (dsl_path[i][2] + zmin) * res;
  }
  return msg; 
}

void DslGrid3D::publishMesh()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time();
  marker.ns = "dsl_grid3d";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 1.0;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  //only if using a MESH_RESOURCE marker type:
  if(use_textured_mesh_)
  {
    marker.color.a = 0.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.mesh_use_embedded_materials = true;

    std::string map_fn(mesh_filename_);
    unsigned int found = map_fn.find_last_of(".");
    std::string texture_fn =  std::string("file://") + map_fn.substr(0,found) + std::string(".dae");
    marker.mesh_resource = texture_fn;
    ROS_INFO("Using textured mesh: %s", texture_fn.c_str());
  }
  else
  {
    marker.mesh_resource = std::string("file://") + std::string(mesh_filename_);
  }

  mesh_marker_pub_.publish( marker );
}

void DslGrid3D::publishOccupancyGrid()
{
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
          pt.x = x/cells_per_meter_;
          pt.y = y/cells_per_meter_;
          pt.z = z/cells_per_meter_;
          marker_pos.push_back(pt);
        }
      }  
    }
  }

  occmap_viz.header.frame_id = "/world";
  occmap_viz.header.stamp = ros::Time();
  occmap_viz.ns = "dsl_grid3d";
  occmap_viz.id = 1;
  occmap_viz.type = visualization_msgs::Marker::CUBE_LIST;
  occmap_viz.action = visualization_msgs::Marker::ADD;
  occmap_viz.pose.position.x = 1.0/(2.*cells_per_meter_) + ogrid_->getPmin()(0);
  occmap_viz.pose.position.y = 1.0/(2.*cells_per_meter_) + ogrid_->getPmin()(1);
  occmap_viz.pose.position.z = 1.0/(2.*cells_per_meter_) + ogrid_->getPmin()(2);
  occmap_viz.pose.orientation.x = 0.0;
  occmap_viz.pose.orientation.y = 0.0;
  occmap_viz.pose.orientation.z = 0.0;
  occmap_viz.pose.orientation.w = 1.0;
  occmap_viz.scale.x = 1.0/cells_per_meter_;
  occmap_viz.scale.y = 1.0/cells_per_meter_;
  occmap_viz.scale.z = 1.0/cells_per_meter_;
  occmap_viz.color.a = 0.5;
  occmap_viz.color.r = 1.0;
  occmap_viz.color.g = 0.0;
  occmap_viz.color.b = 0.0;
  occmap_viz.points = marker_pos;

  occ_map_viz_pub_.publish(occmap_viz);
}

} // namespace
