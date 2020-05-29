#include "dsl_gridsearch/dsl_grid2d.h"
#include "dsl_gridsearch/mesh_utility.h"

#include <limits>

#include <visualization_msgs/Marker.h>

namespace dsl_gridsearch
{

DslGrid2D::DslGrid2D(ros::NodeHandle nh, ros::NodeHandle nh_private) :
  nh_(nh),
  nh_private_(nh_private),
  mesh_filename_("")
{
  double grid_xmin, grid_ymin, grid_zmin;

  if (!nh_private_.getParam ("mesh_filename", mesh_filename_))
    mesh_filename_ = "";
  if (!nh_private_.getParam ("cells_per_meter", cells_per_meter_))
    cells_per_meter_ = 1.;
  if (!nh_private_.getParam ("use_textured_mesh", use_textured_mesh_))
    use_textured_mesh_ = false;
  if (!nh_private_.getParam ("grid_length", grid_length_))
    grid_length_ = -1;
  if (!nh_private_.getParam ("grid_width", grid_width_))
    grid_width_ = -1;
  if (!nh_private_.getParam ("grid_xmin", grid_xmin))
    grid_xmin = 0;
  if (!nh_private_.getParam ("grid_ymin", grid_ymin))
    grid_ymin = 0;


  if(grid_length_ <= 0 || grid_width_ <= 0)
  {
    if(mesh_filename_ == "")
    { 
      ROS_ERROR("Must specify grid bounds if no mesh is specified");
      return;
    }

    ROS_INFO("Loading traversability map from mesh file: %s", mesh_filename_.c_str());
    MeshUtility::meshToHeightMap(mesh_filename_, cells_per_meter_, &ogrid_, DSL_OCCUPIED);
    ROS_INFO("Loaded succesfully");

    mesh_marker_pub_ = nh_.advertise<visualization_msgs::Marker>( "/dsl_grid2d/mesh",  0);
  }
  else
  {
    ROS_INFO("Loading blank traversability map with dimensions: %d %d", grid_length_, grid_width_);
    ogrid_ = new OccupancyGrid(grid_length_, grid_width_, 1, 
                               Eigen::Vector3d(grid_xmin, grid_ymin, 0), 
                               Eigen::Vector3d(grid_length_/cells_per_meter_+grid_xmin, 
                                 grid_width_/cells_per_meter_+grid_ymin, 0), 
                               cells_per_meter_,
                               -100);
  }

  std::cout << "Grid Bounds: " << ogrid_->getPmin().transpose() << " and "
    << ogrid_->getPmax().transpose() << std:: endl;

  occ_map_viz_pub_ = nh_.advertise<visualization_msgs::Marker>( "/dsl_grid2d/trav_map",  0);
  path_pub_ = nh_.advertise<nav_msgs::Path>( "/dsl_grid2d/path",  0);
  optpath_pub_ = nh_.advertise<nav_msgs::Path>( "/dsl_grid2d/optpath",  0);

  ROS_INFO("Building search graph...");
  grid_ = new dsl::Grid2d(ogrid_->getLength(), ogrid_->getWidth(), ogrid_->getOccupancyMap(), 
    1./cells_per_meter_,
    1./cells_per_meter_, 1, DSL_OCCUPIED+1);
  connectivity_ = new dsl::Grid2dConnectivity(*grid_);
  gdsl_ = new dsl::GridSearch<2>(*grid_, *connectivity_, cost_, true);
  gdsl_->SetStart(ogrid_->gridToDslPosition(Eigen::Vector3i(0,0,0)).head<2>());
  gdsl_->SetGoal(ogrid_->gridToDslPosition(Eigen::Vector3i(0,0,0)).head<2>());
  ROS_INFO("Graph built");

  if(grid_length_ > 0 && grid_width_ > 0)
  {
    if(mesh_filename_ != "")
    {
      OccupancyGrid* mesh_grid;
      ROS_INFO("Adding mesh to height map from: %s", mesh_filename_.c_str());
      MeshUtility::meshToHeightMap(mesh_filename_, cells_per_meter_, &mesh_grid, DSL_OCCUPIED);
      ROS_INFO("Loaded succesfully");

      for(int x = 0; x < mesh_grid->getLength(); x++)
      {  
        for(int y = 0; y < mesh_grid->getWidth(); y++)  
        {
          Eigen::Vector3i gp(x,y,0);
          Eigen::Vector3d wp = mesh_grid->gridToPosition(gp);
          Eigen::Vector3i gpos = ogrid_->positionToGrid(wp);
          if(ogrid_->isInGrid(gpos) && mesh_grid->getCost(gp) > ogrid_->getCost(gpos))
          { 
            ogrid_->setCost(mesh_grid->getCost(gp), gpos);
            gdsl_->SetCost(ogrid_->gridToDslPosition(gpos).head<2>(), mesh_grid->getCost(gp)); 
          }
        }
      }
      delete mesh_grid;

      mesh_marker_pub_ = nh_.advertise<visualization_msgs::Marker>( "/dsl_grid2d/mesh",  0);
    }
  } 

  ros::Duration(0.5).sleep();
  //planAllPaths();
  publishAllPaths();
  publishOccupancyGrid();
  if(mesh_filename_ != "")
  {
    publishMesh();
  }

  ROS_INFO("Published occupancy grid");

  set_start_sub_ = nh_.subscribe<geometry_msgs::Point>("/dsl_grid2d/set_start", 10, &DslGrid2D::handleSetStart, this, ros::TransportHints().tcpNoDelay());
  set_goal_sub_ = nh_.subscribe<geometry_msgs::Point>("/dsl_grid2d/set_goal", 10, &DslGrid2D::handleSetGoal, this, ros::TransportHints().tcpNoDelay());
  set_cost_sub_ = nh_.subscribe<geometry_msgs::Point>("/dsl_grid2d/set_cost", 10, &DslGrid2D::handleSetCost, this, ros::TransportHints().tcpNoDelay());
  set_mesh_cost_sub_ = nh_.subscribe<shape_msgs::Mesh>("/dsl_grid2d/set_mesh_cost", 10, &DslGrid2D::handleSetMeshCost, this, ros::TransportHints().tcpNoDelay());
}

void DslGrid2D::handleSetStart(const geometry_msgs::PointConstPtr& msg)
{
  Eigen::Vector3d wpos(msg->x, msg->y, 0);

  if(!isPosInBounds(wpos))
  {
    ROS_WARN("handleSetStart: Position %f %f out of bounds!", wpos(0), wpos(1));
    return;
  }
 
  ROS_INFO("Set start pos: %f %f", wpos(0), wpos(1));
  gdsl_->SetStart(ogrid_->positionToDslPosition(wpos).head<2>());

  planAllPaths();
  publishAllPaths();
}
void DslGrid2D::handleSetGoal(const geometry_msgs::PointConstPtr& msg)
{
  Eigen::Vector3d wpos(msg->x, msg->y, 0);

  if(!isPosInBounds(wpos))
  {
    ROS_WARN("handleSetGoal: Position %f %f out of bounds!", wpos(0), wpos(1));
    return;
  } 

  ROS_INFO("Set goal pos: %f %f", wpos(0), wpos(1));
  gdsl_->SetGoal(ogrid_->positionToDslPosition(wpos).head<2>());

  planAllPaths();
  publishAllPaths();
}
void DslGrid2D::handleSetCost(const geometry_msgs::PointConstPtr& msg)
{
  Eigen::Vector3d wpos(msg->x, msg->y, 0);

  if(!isPosInBounds(wpos))
  {
    ROS_WARN("handleSetOccupied: Position %f %f out of bounds!", wpos(0), wpos(1));
    return;
  } 

  Eigen::Vector3i gpos = ogrid_->positionToGrid(wpos);
  ogrid_->setCost(msg->z, gpos);
  gdsl_->SetCost(ogrid_->positionToDslPosition(wpos).head<2>(), msg->z);

  std::cout << "Set Occupied pos: " << wpos.transpose() << std::endl;

  publishOccupancyGrid();
  planAllPaths();
  publishAllPaths();
}

void DslGrid2D::handleSetMeshCost(const shape_msgs::MeshConstPtr& msg)
{
  OccupancyGrid* new_ogrid;
  MeshUtility::meshToHeightMap(msg, cells_per_meter_, &new_ogrid, DSL_OCCUPIED);
  for(int x = 0; x < new_ogrid->getLength(); x++)
  {  
    for(int y = 0; y < new_ogrid->getWidth(); y++)  
    {
      Eigen::Vector3i gp(x,y,0);
      Eigen::Vector3d wp = new_ogrid->gridToPosition(gp);
      Eigen::Vector3i gpos = ogrid_->positionToGrid(wp);
      if(ogrid_->isInGrid(gpos) && new_ogrid->getCost(gp) > ogrid_->getCost(gpos))
      { 
        ogrid_->setCost(new_ogrid->getCost(gp), gpos);
        gdsl_->SetCost(ogrid_->gridToDslPosition(gpos).head<2>(), new_ogrid->getCost(gp)); 
      }
    }
  }
  delete new_ogrid; 
}

bool DslGrid2D::isPosInBounds(const Eigen::Vector3d& pos)
{
  Eigen::Vector3d pmin = ogrid_->getPmin();
  Eigen::Vector3d pmax = ogrid_->getPmax();
  return (pos(0) >= pmin(0) && pos(1) >= pmin(1) && pos(2) >= pmin(2) && pos(0) <= pmax(0) && pos(1) <= pmax(1) && pos(2) <= pmax(2));
}

void DslGrid2D::planAllPaths()
{
  gdsl_->Plan(path_);

  double maxz = 0;
  for(int i = 0; i < path_.cells.size(); i++)
  {
    std::cout << path_.cells[i].c.transpose() << std::endl;
    if(gdsl_->GetCost(path_.cells[i].c) > maxz)
      maxz = gdsl_->GetCost(path_.cells[i].c); 
  }
  gdsl_->OptPath(path_, optpath_, maxz, 0.1);
  ROS_INFO("Planned paths");
  //gdsl_->SmoothPathSpline(path_, splinepath_, spline_path_maxvelocity_*cells_per_meter_/10., .4);
  //gdsl_->SmoothPathSpline(optpath_, splineoptpath_, spline_path_maxvelocity_*cells_per_meter_/10., .4);
}

void DslGrid2D::publishAllPaths()
{
  path_pub_.publish(dslPathToRosMsg(path_)); 
  optpath_pub_.publish(dslPathToRosMsg(optpath_)); 
  //splinepath_pub_.publish(dslPathToRosMsg(splinepath_)); 
  //splineoptpath_pub_.publish(dslPathToRosMsg(splineoptpath_)); 
}

nav_msgs::Path DslGrid2D::dslPathToRosMsg(const dsl::GridPath<2> &dsl_path)
{
  nav_msgs::Path msg;  
  
  msg.header.frame_id = "/world";
  msg.header.stamp = ros::Time::now();
  msg.poses.resize(dsl_path.cells.size());
  double xmin = ogrid_->getPmin()(0);
  double ymin = ogrid_->getPmin()(1);
  for(int i = 0; i < dsl_path.cells.size(); i++)
  {
    msg.poses[i].pose.position.x = dsl_path.cells[i].c[0] + xmin;//0.5;
    msg.poses[i].pose.position.y = dsl_path.cells[i].c[1] + ymin;//0.5;
    msg.poses[i].pose.position.z = 0;//0.5;
  }
  return msg; 
}

void DslGrid2D::publishMesh()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time();
  marker.ns = "dsl_grid2d";
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

void DslGrid2D::publishOccupancyGrid()
{
  visualization_msgs::Marker occmap_viz;

  // TODO make each square different, color by height? maybe just do boxes
  std::vector<geometry_msgs::Point> marker_pos;
  int length = ogrid_->getLength();
  int width = ogrid_->getWidth();
  for(int x = 0; x < length; x++)
  {
    for(int y = 0; y < width; y++)
    {
       
      int idx = x + y*length;
      assert(!(idx >= length*width || idx < 0));
      geometry_msgs::Point pt;
      pt.x = x/cells_per_meter_;
      pt.y = y/cells_per_meter_;
			//pt.z = gdsl_->GetCost(ogrid_->gridToDslPosition(Eigen::Vector3i(x,y,0)).head<2>())
      //  - ogrid_->getPmin()(2);
      pt.z = ogrid_->getOccupancyMap()[idx];
      marker_pos.push_back(pt);
    }
  }

  occmap_viz.header.frame_id = "/world";
  occmap_viz.header.stamp = ros::Time();
  occmap_viz.ns = "dsl_grid2d";
  occmap_viz.id = 1;
  occmap_viz.type = visualization_msgs::Marker::CUBE_LIST;
  occmap_viz.action = visualization_msgs::Marker::ADD;
  occmap_viz.pose.position.x = 1.0/(2.*cells_per_meter_) + ogrid_->getPmin()(0);
  occmap_viz.pose.position.y = 1.0/(2.*cells_per_meter_) + ogrid_->getPmin()(1);
  occmap_viz.pose.position.z = 0;
  occmap_viz.pose.orientation.x = 0.0;
  occmap_viz.pose.orientation.y = 0.0;
  occmap_viz.pose.orientation.z = 0.0;
  occmap_viz.pose.orientation.w = 1.0;
  occmap_viz.scale.x = 1.0/cells_per_meter_;
  occmap_viz.scale.y = 1.0/cells_per_meter_;
  occmap_viz.scale.z = 0.01;
  occmap_viz.color.a = 0.5;
  occmap_viz.color.r = 1.0;
  occmap_viz.color.g = 0.0;
  occmap_viz.color.b = 0.0;
  occmap_viz.points = marker_pos;

  occ_map_viz_pub_.publish(occmap_viz);
}

} // namespace
