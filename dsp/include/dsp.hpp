#ifndef _DSL_GRID_3D_H_
#define _DSL_GRID_3D_H_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "dsp_interfaces/srv/path_cost.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "dsl/gridsearch.h"
#include "dsl/gridcost.h"
#include "dsl/grid3d.h"
#include "dsl/grid3dconnectivity.h"

#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap/octomap.h"

#include "visualization_msgs/msg/marker.hpp"

#include <memory>
#include <Eigen/Dense>
#include <cmath>

namespace dsp
{

class Dsp : public rclcpp::Node
{
public:
    Dsp();

private:
    // callbac for map
    // 2D
    void occupancy_grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    //3D
    void octomap_data_callback(const octomap_msgs::msg::Octomap::SharedPtr msg);
    void updateGDSP(std::shared_ptr<octomap::OcTree> tree);
    void buildGDSP(std::shared_ptr<octomap::OcTree> tree);      
    void saftyMarginal(Eigen::Vector3d pos, bool update);
    void saftyMarginalFree(Eigen::Vector3d pos);
    void saftyMarginalLoop(Eigen::Vector3d pos);
    void buildGraph();  
    
    // set starting point of planing
    void handleSetStart(const geometry_msgs::msg::Point::SharedPtr msg);
    void handleSetStartOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
    void setStart(Eigen::Vector3d wpos);        
    void setTfStart();

    void request_cost(const std::shared_ptr<dsp_interfaces::srv::PathCost::Request> req, std::shared_ptr<dsp_interfaces::srv::PathCost::Response> res);
    float path_distance(const nav_msgs::msg::Path path);
    float point_distance(geometry_msgs::msg::Point p0, geometry_msgs::msg::Point p1);
    bool setSG(Eigen::Vector3d grid_start, Eigen::Vector3d grid_goal);
    
    // set ending point of planing
    void handleSetGoal(const geometry_msgs::msg::Point::SharedPtr msg);
    void setGoal(Eigen::Vector3d wpos); 
    void setAndPublishPath();
    void publishAllPaths();
    void planAllPaths();
    nav_msgs::msg::Path dspPathToRosMsg(const dsl::GridPath<3>& dsp_path, bool isSplined);
    nav_msgs::msg::Path dspPathToRosMsg(const std::vector<Eigen::Vector3d>& dsp_path, bool isSplined);       
    // Path update
    void pathUpdateCallback();

    void publishOccupancyGrid();
    Eigen::Vector3d posRes(Eigen::Vector3d wpos);               
    
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr occ_map_viz_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr splinepath_pub_;     
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr set_start_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr set_start_odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr set_goal_sub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr get_octomap_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr get_ocupancy_sub_;
    rclcpp::TimerBase::SharedPtr path_update_timer;

    rclcpp::Service<dsp_interfaces::srv::PathCost>::SharedPtr cost_srv_;
    
    // dsp variables
    std::shared_ptr<dsl::Grid3d> grid_;
    dsl::GridCost<3> cost_;
    std::shared_ptr<dsl::Grid3dConnectivity> connectivity_;
    std::shared_ptr<dsl::GridSearch<3>> gdsl_;
    dsl::GridPath<3> path_, splinecells_;
    std::vector<Eigen::Vector3d> splinepath_;
    std::shared_ptr<double[]> occupancy_map;    

    // launch params
    int DSP_OCCUPIED = 2000000000;
    
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

    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
};      

} // namespace

#endif
