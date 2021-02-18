#ifndef _DSL_GRID_3D_H_
#define _DSL_GRID_3D_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <shape_msgs/Mesh.h>
#include <nav_msgs/Path.h>

#include "dsl/gridsearch.h"
#include "dsl/gridcost.h"
#include "dsl/grid3d.h"
#include "dsl/grid3dconnectivity.h"
#include "dsl_gridsearch/occupancy_grid.h"


#include <geometry_msgs/Point.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>

#include <memory>

namespace dsl_gridsearch
{

class DslGrid3D
{
public:
  DslGrid3D(ros::NodeHandle nh, ros::NodeHandle nh_private);

private:
  void handleSetStart(const geometry_msgs::PointConstPtr& msg);
  void handleSetGoal(const geometry_msgs::PointConstPtr& msg);
  void handleSetOccupied(const geometry_msgs::PointConstPtr& msg);
  void handleSetUnoccupied(const geometry_msgs::PointConstPtr& msg);
  //void handleAddMesh(const shape_msgs::MeshConstPtr& msg);
  void spin(const ros::TimerEvent& e);
  void octomap_data_callback(const octomap_msgs::OctomapConstPtr& msg);

  void publishAllPaths();
  void publishMesh();
  void publishOccupancyGrid();

  void planAllPaths();
  nav_msgs::Path dslPathToRosMsg(const dsl::GridPath<3>& dsl_path);
  nav_msgs::Path dslPathToRosMsg(const std::vector<Eigen::Vector3d>& dsl_path);
  bool isPosInBounds(const Eigen::Vector3d& pos);

  //dsl::Grid3d* grid_;
  dsl::GridCost<3> cost_;
  //dsl::Grid3dConnectivity* connectivity_;
  std::shared_ptr<dsl::GridSearch<3>> gdsl_;
  dsl::GridPath<3> path_, optpath_, splinecells_, splineoptcells_;
  std::vector<Eigen::Vector3d> splinepath_, splineoptpath_;
  std::shared_ptr<OccupancyGrid> ogrid_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher occ_map_viz_pub_;
  ros::Publisher mesh_marker_pub_;
  ros::Publisher path_pub_;
  ros::Publisher optpath_pub_;
  ros::Publisher splinepath_pub_;
  ros::Publisher splineoptpath_pub_;

  ros::Subscriber set_start_sub_;
  ros::Subscriber set_goal_sub_;
  ros::Subscriber set_occupied_sub_;
  ros::Subscriber set_unoccupied_sub_;
  ros::Subscriber set_mesh_occupied_sub_;
  ros::Subscriber get_octomap_sub_;

  ros::Timer timer;

  std::string mesh_filename_;
  double cells_per_meter_;
  double spline_step_;
  bool use_textured_mesh_;
  int grid_length_;
  int grid_width_;
  int grid_height_;
//  double res_given;
  double res;
};


} // namespace

#endif
