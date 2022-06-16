#ifndef _DSL_GRID_3D_H_
#define _DSL_GRID_3D_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include "dsp/pathCost.h"

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "dsl/gridsearch.h"
#include "dsl/gridcost.h"
#include "dsl/grid3d.h"
#include "dsl/grid3dconnectivity.h"

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>

#include <visualization_msgs/Marker.h>

#include <memory>
#include <Eigen/Dense>

namespace dsp
{

class Dsp
{
public:
    Dsp(ros::NodeHandle nh, ros::NodeHandle nh_private);

private:
    // callbac for map
    // 2D
    void occupancy_grid_callback(const nav_msgs::OccupancyGridConstPtr& msg);
    //3D
    void octomap_data_callback(const octomap_msgs::OctomapConstPtr& msg);
    void updateGDSP(std::shared_ptr<octomap::OcTree> tree);
    void buildGDSP(std::shared_ptr<octomap::OcTree> tree);      
    void saftyMarginal(Eigen::Vector3d pos, bool update);
    void saftyMarginalFree(Eigen::Vector3d pos);
    void saftyMarginalLoop(Eigen::Vector3d pos);
    void buildGraph();  
    
    // set starting point of planing
    void handleSetStart(const geometry_msgs::PointConstPtr& msg);
    void handleSetStartOdom(const nav_msgs::Odometry msg);
    void setStart(Eigen::Vector3d wpos);        
    void setTfStart();

    bool request_cost(dsp::pathCost::Request &req, dsp::pathCost::Response &res);
    bool setSG(Eigen::Vector3d grid_start, Eigen::Vector3d grid_goal);
    
    // set ending point of planing
    void handleSetGoal(const geometry_msgs::PointConstPtr& msg);
    void setGoal(Eigen::Vector3d wpos); 
    void setAndPublishPath();
    void publishAllPaths();
    void planAllPaths();
    nav_msgs::Path dspPathToRosMsg(const dsl::GridPath<3>& dsp_path, bool isSplined);
    nav_msgs::Path dspPathToRosMsg(const std::vector<Eigen::Vector3d>& dsp_path, bool isSplined);       
    void publishOccupancyGrid();
    Eigen::Vector3d posRes(Eigen::Vector3d wpos);               
    
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;        
    ros::Publisher occ_map_viz_pub_;
    ros::Publisher path_pub_;
    ros::Publisher splinepath_pub_;     
    ros::Subscriber set_start_sub_;
    ros::Subscriber set_start_odom_sub_;
    ros::Subscriber set_goal_sub_;
    ros::Subscriber set_frontier_sub;
    ros::Subscriber get_octomap_sub_;

    ros::ServiceServer cost_srv_;
    
    // dsp variables
    std::shared_ptr<dsl::Grid3d> grid_;
    dsl::GridCost<3> cost_;
    std::shared_ptr<dsl::Grid3dConnectivity> connectivity_;
    std::shared_ptr<dsl::GridSearch<3>> gdsl_;
    dsl::GridPath<3> path_, splinecells_;
    std::vector<Eigen::Vector3d> splinepath_;
    std::shared_ptr<double[]> occupancy_map;    

    // launch params
    double spline_step_;
    bool use_gazebo_odom_;
    bool use_3d_;
    std::string map_topic_;
    std::string odom_topic_;
    std::string odom_frame_id_;
    int DSP_UNKNOWN;
    int DSP_OCCUPIED = 2000000000;
    int risk_;
    int lower_thresh_;
    int upper_thresh_;
    
    int length_voxel = 0;
    int width_voxel = 0;
    int height_voxel = 0;
    double length = -1.0;
    double width = -1.0;
    double height = -1.0;
    double res_octomap;
    
    Eigen::Vector3d start_pos;
    Eigen::Vector3d goal_pos;
    bool grid_built = false;
    Eigen::Vector3d pmin;       

    tf::TransformListener *tran;
    tf::TransformListener listener;
};      

} // namespace

#endif
