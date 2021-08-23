#include "dsl_gridsearch/dsl_grid3d.h"


namespace dsl_gridsearch
{

DslGrid3D::DslGrid3D(ros::NodeHandle nh, ros::NodeHandle nh_private) :
  nh_(nh),
  nh_private_(nh_private)
{
    ROS_INFO("DslGrid3D::DslGrid3D(ros::NodeHandle nh, ros::NodeHandle nh_private)");        

    if (!nh_private_.getParam ("map_topic", map_topic_))
        map_topic_ = "octomap_full";
    if (!nh_private_.getParam ("spline_step_", spline_step_))
        spline_step_ = .1;
    if (!nh_private_.getParam ("lower_thresh", lower_thresh_))
        lower_thresh_ = 59;
    if (!nh_private_.getParam ("upper_thresh", upper_thresh_))
        upper_thresh_ = 60;
    if (!nh_private_.getParam ("risk", risk_))
        risk_ = 2;

    if (!nh_private_.getParam ("use_gazebo_odom", use_gazebo_odom_))
        use_gazebo_odom_ = false;
    if (!nh_private_.getParam ("use_3d", use_3d_))
        use_3d_ = true;
    if (!nh_private_.getParam ("odom_topic", odom_topic_))
        odom_topic_ = "pixy/truth/NWU";
    if (!nh_private_.getParam ("odom_frame_id", odom_frame_id_))
        odom_frame_id_ = "odom";
    if (!nh_private_.getParam ("unknown_value", DSL_UNKNOWN))
        DSL_UNKNOWN = 10000;

    occ_map_viz_pub_ = nh_.advertise<visualization_msgs::Marker>( "dsl_grid3d/occupancy_map",  0);
    path_pub_ = nh_.advertise<nav_msgs::Path>( "dsl_grid3d/path",  0);
    splinepath_pub_ = nh_.advertise<nav_msgs::Path>( "dsl_grid3d/splinepath",  0);

    if(use_gazebo_odom_)
    {
        ROS_INFO("Using odom ass start");
        set_start_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic_, 1, 
          &DslGrid3D::handleSetStartOdom, this);
    }
    else
    {
        ROS_INFO("Manualy set start");
        set_start_sub_ = nh_.subscribe<geometry_msgs::Point>("dsl_grid3d/set_start", 1, 
          &DslGrid3D::handleSetStart, this);
    }

    if(use_3d_){
        get_octomap_sub_ = nh_.subscribe<octomap_msgs::Octomap>(map_topic_, 1,
            &DslGrid3D::octomap_data_callback, this);
    }
    else {
        get_octomap_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(map_topic_, 1,
            &DslGrid3D::occupancy_grid_callback, this);
        height_metric = 1;
    }

    set_goal_sub_ = nh_.subscribe<geometry_msgs::Point>("dsl_grid3d/set_goal", 1,
        &DslGrid3D::handleSetGoal, this);
    set_frontier_sub = nh_.subscribe<exploration::Frontier>("next_frontier", 1,
        &DslGrid3D::handleSetFrontier, this);



    start_pos << 0,0,0; 
    goal_pos << 0,0,0; 
}

// 2D occupancyGrid callback 
void DslGrid3D::occupancy_grid_callback(const nav_msgs::OccupancyGridConstPtr& msg){

    if (length * width == msg->info.width * msg->info.height){
        int size = length_metric * width_metric;
        for (int i = 0; i < size; i++){
            if(msg->data[i] > upper_thresh_ && occupancy_map[i] != DSL_OCCUPIED){
                Eigen::Vector3d pos(i % length_metric, i / length_metric, 0);
                saftyMarginal(pos, true);
                occupancy_map[i] = DSL_OCCUPIED;
                gdsl_->SetCost(pos, DSL_OCCUPIED);
            }
            else if (msg->data[i] < lower_thresh_ && msg->data[i] != -1 && occupancy_map[i] >= DSL_UNKNOWN){
                if (occupancy_map[i] == DSL_OCCUPIED){
                    Eigen::Vector3d pos(i % length_metric, i / length_metric, 0);
                    gdsl_->SetCost(pos, 1);
                    occupancy_map[i] = 1;
                    saftyMarginalFree(pos);
                } else {
                    Eigen::Vector3d pos(i % length_metric, i / length_metric, 0);
                    gdsl_->SetCost(pos, occupancy_map[i] - DSL_UNKNOWN + 1);
                    occupancy_map[i] = occupancy_map[i] - DSL_UNKNOWN + 1;
                }
            }
        }
    }
    else {
    
        width = msg->info.height;
        length = msg->info.width;
        res_octomap = msg->info.resolution;
        grid_built = true;
        pmin(0) = msg->info.origin.position.x;
        pmin(1) = msg->info.origin.position.y;
        pmin(2) = 0;

        length_metric = length; 
        width_metric = width; 
        int size = length_metric * width_metric;
        occupancy_map.reset(new double[size]);

        for(int i = 0; i < size; i++){
            occupancy_map[i] = DSL_UNKNOWN; 
        }

        for(int i = 0; i < size; i++){
            if(msg->data[i] > upper_thresh_){
                Eigen::Vector3d pos(i % length_metric, i / length_metric, 0);
                saftyMarginal(pos, false);
                occupancy_map[i] = DSL_OCCUPIED;
            }
            else if (msg->data[i] < lower_thresh_ && msg->data[i] != -1 && occupancy_map[i] >= DSL_UNKNOWN){
                occupancy_map[i] = occupancy_map[i] - DSL_UNKNOWN + 1;
            }
                
        }
        buildGraph();
    }
    publishOccupancyGrid();
    return;
}

// octomap 3D callback
void DslGrid3D::octomap_data_callback(const octomap_msgs::OctomapConstPtr& msg) {
    std::shared_ptr<octomap::OcTree> tree(dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg)));
    res_octomap = tree->getResolution();
    double length_test, width_test, height_test;
    tree->getMetricSize(length_test, width_test, height_test);
    tree->getMetricMin(pmin(0), pmin(1), pmin(2));

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
    //setAndPublishPath();
}


void DslGrid3D::buildGDSL(std::shared_ptr<octomap::OcTree> tree)
{
    int count = 0;
    length_metric = length/res_octomap;
    width_metric = width/res_octomap;
    height_metric = height/res_octomap;

    int size = length_metric * width_metric * height_metric;
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
        // octomap may have lage nodes. loops will add voxels for entire large node
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


    //Bound occupancy_map from top and bottom
/*      for(int x = 0; x < ogrid_->getLength(); x++)
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
  buildGraph();
}

void DslGrid3D::buildGraph(){
    ROS_INFO("Building search graph...");
    grid_.reset(new dsl::Grid3d(length_metric, width_metric, height_metric, 
        occupancy_map.get(),
        1, 1, 1, 1, DSL_OCCUPIED + 1));
    connectivity_.reset(new dsl::Grid3dConnectivity(*grid_));
    gdsl_.reset(new dsl::GridSearch<3>(*grid_, *connectivity_, cost_, true));
    ROS_INFO("Graph built");
}

void DslGrid3D::updateGDSL(std::shared_ptr<octomap::OcTree> tree)
{
    for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it)
    {
        Eigen::Vector3d pos(it.getX(), it.getY(), it.getZ());
        pos = posRes(pos);

            
        int n = it.getSize() / res_octomap;
        int i = -n/2;
	it->getOccupancy();
        // octomap may have lage nodes. loops will add voxels for entire large node
        do{
            int j = -n/2;
            do{
                int k = -n/2;
                do{
	            int idx = ((int) pos(0) + i) + ((int) pos(1) + j) *length_metric + ((int) pos(2) + k) *length_metric*width_metric;
                    Eigen::Vector3d p(pos(0) + i, pos(1) + j, pos(2) + k);

                    if(tree->isNodeOccupied(*it) and occupancy_map[idx] != DSL_OCCUPIED)
                    {
                        saftyMarginal(p, true);
                        gdsl_->SetCost(p, DSL_OCCUPIED);
                        occupancy_map[idx] = DSL_OCCUPIED;
                    }			
                    else if(!tree->isNodeOccupied(*it)  and occupancy_map[idx] >= DSL_UNKNOWN)
                    {
                        if(occupancy_map[idx] < DSL_OCCUPIED)
                        {
                            gdsl_->SetCost(p, occupancy_map[idx] - DSL_UNKNOWN + 1);
                            occupancy_map[idx] = occupancy_map[idx] - DSL_UNKNOWN + 1;
                        }
                        else
                        {
                            gdsl_->SetCost(p, 1);
                            occupancy_map[idx] = 1;
                            saftyMarginalFree(pos);
                        }
                    }
                    k++;
                } while (k < n/2);
                j++;
            } while (j < n/2); 
            i++;
        } while ( i < n/2);
    }
    return;
}

// adding of safty margianl to occupied space inprove safty
void DslGrid3D::saftyMarginal(Eigen::Vector3d pos, bool update)
{
    Eigen::Vector3d local_pose;
    for(int i = -risk_; i <= risk_; i++){
        for(int j = -risk_; j <= risk_; j++){
            for(int k = -risk_; k <= risk_; k++){
                int x = pos(0) + i;
                int y = pos(1) + j;
                int z = pos(2) + k;
                if(x >= 0 && x < length_metric && y >= 0 && y < width_metric && z >= 0 && z < height_metric){
	            int idx = x + y*length_metric + z*length_metric*width_metric;
                    local_pose << pos(0) + i, pos(1) + j, pos(2) + k;
                    int sum = i * i + j * j + k * k;
                    if (!sum){ // sum = 0 > current ocupied space
                        continue;
                    }
                    int cost = DSL_UNKNOWN / (sum + 2);
                    if(occupancy_map[idx] < cost){
                        occupancy_map[idx] = cost;
                        if(update){
                            gdsl_->SetCost(local_pose, cost);
                        }
                    }
                    else if (DSL_UNKNOWN == occupancy_map[idx] or (occupancy_map[idx] >= DSL_UNKNOWN and occupancy_map[idx] < DSL_UNKNOWN + cost)){
                        cost += DSL_UNKNOWN;
                        occupancy_map[idx] = cost;
                        if(update){
                            gdsl_->SetCost(local_pose, cost);
                        }
                    }
                }
            }
        }
    }
}

// adding safty marginal to cells descoverd free
void DslGrid3D::saftyMarginalFree(Eigen::Vector3d pos)
{
    int sum = risk_ * risk_ * 3 + 1;
    int index;
    Eigen::Vector3d local_pose;
    for(int i = -risk_; i <= risk_; i++){
        for(int j = -risk_; j <= risk_; j++){
            for(int k = -risk_; k <= risk_; k++){
                int x = pos(0) + i;
                int y = pos(1) + j;
                int z = pos(2) + k;
                if(x >= 0 && x < length_metric && y >= 0 && y < width_metric && z >= 0 && z < height_metric){
	            int idx = x + y*length_metric + z*length_metric*width_metric;
                    if (occupancy_map[idx] == DSL_OCCUPIED){
                        if (i * i + j * j + k * k < sum){
                            sum = i * i + j * j + k * k;
                            local_pose << pos(0) + i, pos(1) + j, pos(2) + k;
                            index = idx;
                        }
                    }
                }
            }
        }
    }
    if (sum <= risk_ * risk_ * 3){
        int cost = DSL_UNKNOWN / (sum + 2);
        occupancy_map[index] = cost;
        gdsl_->SetCost(local_pose, cost);

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
    if(!use_3d_){
        wpos[2] = 0;
    }
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
    if(!use_3d_){
        wpos[2] = 0;
    }
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

// transfomre a pose betewn IRL cordinates and map cordinate 
Eigen::Vector3d DslGrid3D::posRes(Eigen::Vector3d wpos)
{
    for (int i = 0; i < 3; i++)
    {
        wpos(i) = (wpos(i) - pmin(i)) / res_octomap;
    }
    return wpos;
}

void DslGrid3D::planAllPaths()
{
    gdsl_->Plan(path_);
    //gdsl_->SplinePath(path_, splinepath_, spline_step_);
    return;
}

void DslGrid3D::publishAllPaths()
{
  path_pub_.publish(dslPathToRosMsg(path_, false));
  //splinepath_pub_.publish(dslPathToRosMsg(splinepath_, true)); 
}

// transfom paht to ros paht
nav_msgs::Path DslGrid3D::dslPathToRosMsg(const dsl::GridPath<3> &dsl_path, bool isSplined)
{
  std::vector<Eigen::Vector3d> path;
  for(int i = 0; i < dsl_path.cells.size(); i++)
  {
    path.push_back(dsl_path.cells[i].c * res_octomap);
  }
  return dslPathToRosMsg(path, isSplined);
}

nav_msgs::Path DslGrid3D::dslPathToRosMsg(const std::vector<Eigen::Vector3d> &dsl_path, bool isSplined)
{
  nav_msgs::Path msg;  
  
  msg.header.frame_id = odom_frame_id_; 
  msg.poses.resize(dsl_path.size());
  for(int i = 0; i < dsl_path.size(); i++)
  {
    if(isSplined){
        msg.poses[i].pose.position.x = dsl_path[i][0] * res_octomap + pmin(0);
        msg.poses[i].pose.position.y = dsl_path[i][1] * res_octomap + pmin(1);
        msg.poses[i].pose.position.z = dsl_path[i][2] * res_octomap + pmin(2);
    }
    else{
        msg.poses[i].pose.position.x = dsl_path[i][0] + pmin(0);
        msg.poses[i].pose.position.y = dsl_path[i][1] + pmin(1);
        msg.poses[i].pose.position.z = dsl_path[i][2] + pmin(2);
    }
  }

  return msg; 
}

// publishing map usedfule when debuging
void DslGrid3D::publishOccupancyGrid()
{
    visualization_msgs::Marker occmap_viz;

    std::vector<geometry_msgs::Point> marker_pos;
    std::vector<geometry_msgs::Point> risk_pos;
     
    for(double x = 0; x < length_metric; x++)
    {
        for(double y = 0; y < width_metric; y++)
        {
            for(double z = 0; z < height_metric; z++)
            {
                Eigen::Vector3d pos(x, y, z);
                int idx = x + y*length + z*length*width;

                // Different parts to vizulize
                //if(gdsl_->GetCost(pos) == DSL_OCCUPIED)
                //if(gdsl_->GetCost(pos) == DSL_UNKNOWN)
                //if(gdsl_->GetCost(pos) == 1)
                if(gdsl_->GetCost(pos) > 1 and gdsl_->GetCost(pos) < DSL_UNKNOWN)
                //if(gdsl_->GetCost(pos) < DSL_OCCUPIED and gdsl_->GetCost(pos) > DSL_UNKNOWN)
                //if(gdsl_->GetCost(pos) >= DSL_UNKNOWN / 5 and gdsl_->GetCost(pos) < DSL_UNKNOWN)
                {
                    geometry_msgs::Point pt;
                    pt.x = (x + pmin(0)) * res_octomap;
                    pt.y = (y + pmin(1)) * res_octomap;
                    pt.z = (z + pmin(2)) * res_octomap;
                    marker_pos.push_back(pt);
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
    occ_map_viz_pub_.publish(occmap_viz);

}

} // namespace
