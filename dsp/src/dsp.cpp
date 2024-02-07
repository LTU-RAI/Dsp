#include "dsp.hpp"


namespace dsp
{

Dsp::Dsp() : Node("dsp")
{
    this->declare_parameter("map_topic", "octomap_full");
    this->declare_parameter("spline_step", 0.01);
    this->declare_parameter("lower_thresh", 59);
    this->declare_parameter("upper_thresh", 60);
    this->declare_parameter("risk", 2);
    this->declare_parameter("RATE", 2000.0);
    this->declare_parameter("use_odometry", false);
    this->declare_parameter("use_3d", true);
    this->declare_parameter("odom_topic", "odometry/imu");
    this->declare_parameter("odom_frame_id", "map");
    this->declare_parameter("base_link_frame_id", "base_link");
    this->declare_parameter("DSP_UNKNOWN", 10000);
    this->declare_parameter("update_rate", 1);
    this->declare_parameter("debug", false);



    occ_map_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("dsp/occupancy_map",1);

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("dsp/path", 1);
    splinepath_pub_ =  this->create_publisher<nav_msgs::msg::Path>("dsp/spline_path", 1);

    if(this->get_parameter("use_odometry").get_parameter_value().get<bool>())
    {
        set_start_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("odom_topic").get_parameter_value().get<std::string>(), 1, 
            std::bind(&Dsp::handleSetStartOdom, this, std::placeholders::_1));
    }
    else
    {
        set_start_sub_ = this->create_subscription<geometry_msgs::msg::Point>("dsp/set_start", 1,
            std::bind(&Dsp::handleSetStart, this, std::placeholders::_1));
    }

    if(this->get_parameter("use_3d").get_parameter_value().get<bool>()){
        get_octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            this->get_parameter("map_topic").get_parameter_value().get<std::string>(), 1,
            std::bind(&Dsp::octomap_data_callback, this, std::placeholders::_1));
    }
    else {
        get_ocupancy_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(this->get_parameter("map_topic").get_parameter_value().get<std::string>(), 1,
            std::bind(&Dsp::occupancy_grid_callback, this, std::placeholders::_1));
        height_voxel = 1;
    }

    set_goal_sub_ = this->create_subscription<geometry_msgs::msg::Point>("dsp/set_goal", 1,
        std::bind(&Dsp::handleSetGoal, this, std::placeholders::_1));

    double RATE = this->get_parameter("RATE").get_parameter_value().get<double>();
    if(RATE > 0.0)
        path_update_timer = this->create_wall_timer(std::chrono::duration<double, std::milli>(RATE), std::bind(&Dsp::pathUpdateCallback, this));



    cost_srv_ = this->create_service<dsp_interfaces::srv::PathCost>("dsp/path_cost", 
    //&Dsp::request_cost, this);
    std::bind(&Dsp::request_cost, this, std::placeholders::_1, std::placeholders::_2));

    start_pos << 0,0,0; 
    goal_pos << 0,0,0; 

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
}

// 2D occupancyGrid callback 
void Dsp::occupancy_grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){

    int DSP_UNKNOWN = this->get_parameter("DSP_UNKNOWN").get_parameter_value().get<int>();
    int upper_thresh_ = this->get_parameter("upper_thresh").get_parameter_value().get<int>();
    int lower_thresh_ = this->get_parameter("lower_thresh").get_parameter_value().get<int>();

    pmin(0) = msg->info.origin.position.x;
    pmin(1) = msg->info.origin.position.y;
    pmin(2) = 0;

    if (length * width == msg->info.width * msg->info.height){
        int size = length_voxel * width_voxel;
        for (int i = 0; i < size; i++){
            Eigen::Vector3d pos(i % length_voxel, i / length_voxel, 0);
            if(msg->data[i] > upper_thresh_ && occupancy_map[i] != DSP_OCCUPIED){
                saftyMarginal(pos, true);
                occupancy_map[i] = DSP_OCCUPIED;
                gdsl_->SetCost(pos, DSP_OCCUPIED);
            }
            else if (msg->data[i] < lower_thresh_ && msg->data[i] != -1 && occupancy_map[i] >= DSP_UNKNOWN){
                if (occupancy_map[i] == DSP_OCCUPIED){
                    gdsl_->SetCost(pos, 1);
                    occupancy_map[i] = 1;
                    saftyMarginalLoop(pos);
                } else {
                    gdsl_->SetCost(pos, occupancy_map[i] - DSP_UNKNOWN + 1);
                    occupancy_map[i] = occupancy_map[i] - DSP_UNKNOWN + 1;
                }
            }
            else if (msg->data[i] == -1){
                if (occupancy_map[i] < DSP_UNKNOWN){
                    gdsl_->SetCost(pos, occupancy_map[i] + DSP_UNKNOWN);
                    occupancy_map[i] = occupancy_map[i] + DSP_UNKNOWN;
                }
                else if (occupancy_map[i] == DSP_OCCUPIED){
                    gdsl_->SetCost(pos, DSP_UNKNOWN);
                    occupancy_map[i] = DSP_UNKNOWN;
                    saftyMarginalLoop(pos);
                }
            }
        }
    }
    else {
    
        width = msg->info.height;
        length = msg->info.width;
        res_octomap = msg->info.resolution;
        grid_built = true;

        length_voxel = length; 
        width_voxel = width; 
        int size = length_voxel * width_voxel;
        occupancy_map.reset(new double[size]);

        for(int i = 0; i < size; i++){
            occupancy_map[i] = DSP_UNKNOWN; 
        }

        for(int i = 0; i < size; i++){
            if(msg->data[i] > upper_thresh_){
                Eigen::Vector3d pos(i % length_voxel, i / length_voxel, 0);
                saftyMarginal(pos, false);
                occupancy_map[i] = DSP_OCCUPIED;
            }
            else if (msg->data[i] < lower_thresh_ && msg->data[i] != -1 && occupancy_map[i] >= DSP_UNKNOWN){
                occupancy_map[i] = occupancy_map[i] - DSP_UNKNOWN + 1;
            }
                
        }
        buildGraph();
    }
    //publishOccupancyGrid();
    //setAndPublishPath();
    return;
}

// octomap 3D callback
void Dsp::octomap_data_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    std::shared_ptr<octomap::OcTree> tree(dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg)));
    res_octomap = tree->getResolution();
    double length_test, width_test, height_test;
    tree->getMetricSize(length_test, width_test, height_test);
    tree->getMetricMin(pmin(0), pmin(1), pmin(2));

    if (length_test * width_test * height_test == length * width * height)
    {
        updateGDSP(tree);
    }
    else
    {

        length = length_test;
        width = width_test;
        height = height_test;
        buildGDSP(tree);
        grid_built = true;
    }
    //publishOccupancyGrid();
    //setAndPublishPath();
}


void Dsp::buildGDSP(std::shared_ptr<octomap::OcTree> tree)
{
    int DSP_UNKNOWN = this->get_parameter("DSP_UNKNOWN").get_parameter_value().get<int>();
    
    length_voxel = round(length/res_octomap);
    width_voxel = round(width/res_octomap);
    height_voxel = round(height/res_octomap);

    int size = length_voxel * width_voxel * height_voxel;
    occupancy_map.reset(new double[size]);

   for(int i = 0; i < size; i++)
   {
     occupancy_map[i] = DSP_UNKNOWN; 
   }

   for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it)
    {
        Eigen::Vector3d pos(it.getX(), it.getY(), it.getZ());
        pos = posRes(pos);
        int n = it.getSize() / res_octomap;
        
        if (n != 1){
            for (int i = 0; i < 3; i++){
                pos(i) = pos(i) + 0.5;
            }
        }
        int i = -n/2;
	//it->getOccupancy();
        // octomap may have lage nodes. loops will add voxels for entire large node
        do{
            int j = -n/2;
            do{
                int k = -n/2;
                do{
	            int idx = ((int) pos(0) + i) + (((int) pos(1) + j) * length_voxel) + (((int) pos(2) + k) * length_voxel * width_voxel);
	            if(tree->isNodeOccupied(*it)) {
                        Eigen::Vector3d p(pos(0) + i, pos(1) + j, pos(2) + k);
                        saftyMarginal(p, false);
		        occupancy_map[idx] = DSP_OCCUPIED;
	            }
                    else {//if (occupancy_map[idx] >= DSP_UNKNOWN){
                        occupancy_map[idx] = occupancy_map[idx] - DSP_UNKNOWN + 1;
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
        for(int y = 0; y < width_voxel; y++)  
        {
            //for(int z = 0; z < height_voxel; z++)  
            //{
                Eigen::Vector3i gpMin(x, y, zmin);
                Eigen::Vector3d wpMin = ogrid_->gridToPosition(gpMin);
                ogrid_->setOccupied(wpMin, true);

                Eigen::Vector3i gpMax(x, y, height_voxel-1);
                Eigen::Vector3d wpMax = ogrid_->gridToPosition(gpMax);
                ogrid_->setOccupied(wpMax, true);
            //}
        }
    }
*/
  buildGraph();
}

void Dsp::buildGraph(){
    RCLCPP_INFO(this->get_logger(), "Building search graph...");
    //int size = length_voxel * width_voxel * height_voxel;
    grid_.reset(new dsl::Grid3d(length_voxel, width_voxel, height_voxel, 
        occupancy_map.get(),
        1, 1, 1, 1, DSP_OCCUPIED + 1));
    connectivity_.reset(new dsl::Grid3dConnectivity(*grid_));
    gdsl_.reset(new dsl::GridSearch<3>(*grid_, *connectivity_, cost_, true));
    RCLCPP_INFO(this->get_logger(), "Graph built");
}

void Dsp::updateGDSP(std::shared_ptr<octomap::OcTree> tree)
{
    int DSP_UNKNOWN = this->get_parameter("DSP_UNKNOWN").get_parameter_value().get<int>();

    for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it)
    {
        Eigen::Vector3d pos(it.getX(), it.getY(), it.getZ());
        pos = posRes(pos);

            
        int n = it.getSize() / res_octomap;
        if (n != 1){
            for (int i = 0; i < 3; i++){
                pos(i) = pos(i) + 0.5;
            }
        }
        int i = -n/2;
	//it->getOccupancy();
        // octomap may have lage nodes. loops will add voxels for entire large node
        do{
            int j = -n/2;
            do{
                int k = -n/2;
                do{
	            int idx = ((int) pos(0) + i) + ((int) pos(1) + j) *length_voxel + ((int) pos(2) + k) *length_voxel*width_voxel;
                    Eigen::Vector3d p(pos(0) + i, pos(1) + j, pos(2) + k);

                    if(tree->isNodeOccupied(*it) and occupancy_map[idx] != DSP_OCCUPIED)
                    {
                        gdsl_->SetCost(p, DSP_OCCUPIED);
                        occupancy_map[idx] = DSP_OCCUPIED;
                        saftyMarginal(p, true);
                    }			
                    else if(!tree->isNodeOccupied(*it)  and occupancy_map[idx] >= DSP_UNKNOWN)
                    {
                        if(occupancy_map[idx] < DSP_OCCUPIED)
                        {
                            gdsl_->SetCost(p, occupancy_map[idx] - DSP_UNKNOWN + 1);
                            occupancy_map[idx] = occupancy_map[idx] - DSP_UNKNOWN + 1;
                        }
                        else
                        {
                            gdsl_->SetCost(p, 1);
                            occupancy_map[idx] = 1;
                            saftyMarginalLoop(pos);
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
void Dsp::saftyMarginal(Eigen::Vector3d pos, bool update)
{
    int DSP_UNKNOWN = this->get_parameter("DSP_UNKNOWN").get_parameter_value().get<int>();
    int risk_ = this->get_parameter("risk").get_parameter_value().get<int>();

    Eigen::Vector3d local_pose;
    for(int i = -risk_; i <= risk_; i++){
        for(int j = -risk_; j <= risk_; j++){
            for(int k = -risk_; k <= risk_; k++){
                int x = pos(0) + i;
                int y = pos(1) + j;
                int z = pos(2) + k;
                if(x >= 0 && x < length_voxel && y >= 0 && y < width_voxel && z >= 0 && z < height_voxel){
	            int idx = x + y*length_voxel + z*length_voxel*width_voxel;
                    local_pose << pos(0) + i, pos(1) + j, pos(2) + k;
                    int sum = i * i + j * j + k * k;
                    if (!sum){ // sum = 0 > current ocupied space
                        continue;
                    }
                    int cost = DSP_UNKNOWN / (sum + 2);
                    if(occupancy_map[idx] < cost){
                        occupancy_map[idx] = cost;
                        if(update){
                            gdsl_->SetCost(local_pose, cost);
                        }
                    }
                    else if (DSP_UNKNOWN == occupancy_map[idx] or (occupancy_map[idx] >= DSP_UNKNOWN and occupancy_map[idx] < DSP_UNKNOWN + cost)){
                        cost += DSP_UNKNOWN;
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
void Dsp::saftyMarginalFree(Eigen::Vector3d pos)
{
    int risk_ = this->get_parameter("risk").get_parameter_value().get<int>();
    int DSP_UNKNOWN = this->get_parameter("DSP_UNKNOWN").get_parameter_value().get<int>();
    int sum = risk_ * risk_ * 3 + 1;
    int index;
    Eigen::Vector3d local_pose;
    for(int i = -risk_; i <= risk_; i++){
        for(int j = -risk_; j <= risk_; j++){
            for(int k = -risk_; k <= risk_; k++){
                int x = (int)round(pos(0)) + i;
                int y = (int)round(pos(1)) + j;
                int z = (int)round(pos(2)) + k;
                if(x >= 0 && x < length_voxel && y >= 0 && y < width_voxel && z >= 0 && z < height_voxel){
	            int idx = x + y*length_voxel + z*length_voxel*width_voxel;
                    if (occupancy_map[idx] == DSP_OCCUPIED){
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
    int idx = pos(0) + pos(1)*length_voxel + pos(2)*length_voxel*width_voxel;
    if (idx < 0 or idx > (length_voxel * width_voxel * height_voxel)){
	//return;
    }
    if (sum <= risk_ * risk_ * 3){
        int cost = DSP_UNKNOWN / (sum + 2);
        if (occupancy_map[index] < DSP_UNKNOWN){
            if(occupancy_map[idx] < cost){
                occupancy_map[idx] = cost;
                gdsl_->SetCost(pos, cost);
            }
        }
        else if (occupancy_map[idx] >= DSP_UNKNOWN && occupancy_map[index] != DSP_OCCUPIED){
            cost = cost + DSP_UNKNOWN;
            if (cost > occupancy_map[idx]){
                occupancy_map[idx] = cost;
                gdsl_->SetCost(pos, cost);
            }
        }

    } else {
        if (occupancy_map[idx] < DSP_UNKNOWN){
            gdsl_->SetCost(pos, 1);
            occupancy_map[idx] = 1;
        } else {
            gdsl_->SetCost(pos, DSP_UNKNOWN);
            occupancy_map[idx] = DSP_UNKNOWN;
        }
    }
}

void Dsp::saftyMarginalLoop(Eigen::Vector3d pos){
    
    int risk_ = this->get_parameter("risk").get_parameter_value().get<int>();
    Eigen::Vector3d local_pose;
    for(int i = -risk_; i <= risk_; i++){
        for(int j = -risk_; j <= risk_; j++){
            for(int k = -risk_; k <= risk_; k++){
                int x = (int)round(pos(0)) + i;
                int y = (int)round(pos(1)) + j;
                int z = (int)round(pos(2)) + k;
                if(x >= 0 && x < length_voxel && y >= 0 && y < width_voxel && z >= 0 && z < height_voxel){
                    local_pose << x, y, z;
                    saftyMarginalFree(local_pose);
                }
            }
        }
    }
    
}

void Dsp::handleSetStartOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    Eigen::Vector3d wpos(msg->pose.pose.position.x , msg->pose.pose.position.y, msg->pose.pose.position.z);
    //setStart(wpos);
    //setAndPublishPath();

    
}

void Dsp::request_cost(const std::shared_ptr<dsp_interfaces::srv::PathCost::Request> req, std::shared_ptr<dsp_interfaces::srv::PathCost::Response> res){

    Eigen::Vector3d start_req(req->start.x, req->start.y, req->start.z);
    Eigen::Vector3d goal_req(req->stop.x, req->stop.y, req->stop.z);


    if(!setSG(start_req, goal_req)){
        return;
    }

    dsl::GridPath<3> path;
    gdsl_->Plan(path);
    res->cost = path.cost;
    nav_msgs::msg::Path ros_path = dspPathToRosMsg(path,false);
    res->path = ros_path;
    res->distance = path_distance(ros_path);

    return;

}

float Dsp::path_distance(const nav_msgs::msg::Path path)
{
    float distance = 0.0;
    for(unsigned long int i = 0; i + 1 < path.poses.size(); i++)
    {
        distance += point_distance(path.poses[i].pose.position, path.poses[i+1].pose.position);
    }
    return distance;
}

float Dsp::point_distance( geometry_msgs::msg::Point p0,  geometry_msgs::msg::Point p1)
{
    return sqrt(pow(p0.x - p1.x, 2) + pow(p0.y - p1.y, 2) + pow(p0.z - p1.z, 2));
}

void Dsp::handleSetStart(const geometry_msgs::msg::Point::SharedPtr msg)
{
    Eigen::Vector3d wpos(msg->x, msg->y, msg->z);
    setStart(wpos);
}

void Dsp::setTfStart(){
    std::string odf = this->get_parameter("odom_frame_id").get_parameter_value().get<std::string>();
    std::string blf = this->get_parameter("base_link_frame_id").get_parameter_value().get<std::string>();
    geometry_msgs::msg::TransformStamped t;
    try{
        t = tf_buffer->lookupTransform(
        odf, blf, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN( this->get_logger(),
            "Could not transform %s to %s: %s",
            odf.c_str(), blf.c_str(), ex.what());
        return;
    }

    Eigen::Vector3d wpos(t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z);
    if(!this->get_parameter("use_3d").get_parameter_value().get<bool>())
        wpos[2] = 0.0;
    setStart(wpos);
}

void Dsp::setStart(Eigen::Vector3d wpos)
{
    if(!this->get_parameter("use_3d").get_parameter_value().get<bool>()){
        wpos[2] = 0;
    }
    start_pos = wpos; 
    //setAndPublishPath();
}

void Dsp::handleSetGoal(const geometry_msgs::msg::Point::SharedPtr msg)
{
    Eigen::Vector3d wpos(msg->x, msg->y, msg->z);
    setGoal(wpos);
}

void Dsp::setGoal(Eigen::Vector3d wpos)
{
    if(!this->get_parameter("use_3d").get_parameter_value().get<bool>()){
        wpos[2] = 0;
    }
    goal_pos = wpos; 
    setAndPublishPath();
}

void Dsp::setAndPublishPath(){
    if(!grid_built){
        return;
    }
    setTfStart();
    //Eigen::Vector3d grid_start = posRes(start_pos);
    //Eigen::Vector3d grid_goal = posRes(goal_pos);

    if(!setSG(start_pos, goal_pos)){
        return;
    }
    /*
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
    */

    planAllPaths();
    publishAllPaths();

}

bool Dsp::setSG(Eigen::Vector3d start, Eigen::Vector3d goal){

    Eigen::Vector3d grid_start = posRes(start);
    Eigen::Vector3d grid_goal = posRes(goal);

    if((int) grid_start(0) == (int) grid_goal(0)
        and (int) grid_start(1) == (int) grid_goal(1)
        and (int) grid_start(2) == (int) grid_goal(2))
    {
        RCLCPP_WARN(this->get_logger(), "Start and goal poses are the some");
        return false;
    }
    if (!gdsl_->SetStart(grid_start))
    {
        RCLCPP_WARN(this->get_logger(), "SetStart faild");
        return false;
    }
    if (!gdsl_->SetGoal(grid_goal))
    {
        RCLCPP_WARN(this->get_logger(), "SetGoal faild");
        return false;
    }
    return true;
}
// transfomre a pose betewn IRL cordinates and map cordinate 
Eigen::Vector3d Dsp::posRes(Eigen::Vector3d wpos)
{
    for (int i = 0; i < 3; i++)
    {
        wpos(i) = (wpos(i) - pmin(i)) / res_octomap;
    }
    return wpos;
}

void Dsp::pathUpdateCallback(){
    setAndPublishPath();
}

void Dsp::planAllPaths()
{
    gdsl_->Plan(path_);
    //gdsl_->SplinePath(path_, splinepath_, spline_step_);
    return;
}

void Dsp::publishAllPaths()
{
  path_pub_->publish(dspPathToRosMsg(path_, false));
  //if (path_.cells.size() <= 3){
  //  splinepath_pub_.publish(dspPathToRosMsg(path_, false)); 
  //} else {
  //  splinepath_pub_.publish(dspPathToRosMsg(splinepath_, true)); 
  //}
}

// transfom paht to ros paht
nav_msgs::msg::Path Dsp::dspPathToRosMsg(const dsl::GridPath<3> &dsp_path, bool isSplined)
{
  std::vector<Eigen::Vector3d> path;
  for(unsigned long int i = 0; i < dsp_path.cells.size(); i++)
  {
    path.push_back(dsp_path.cells[i].c * res_octomap);
  }
  return dspPathToRosMsg(path, isSplined);
}

nav_msgs::msg::Path Dsp::dspPathToRosMsg(const std::vector<Eigen::Vector3d> &dsp_path, bool isSplined)
{
  nav_msgs::msg::Path msg;  
  
  msg.header.frame_id = this->get_parameter("odom_frame_id").get_parameter_value().get<std::string>(); 
  msg.poses.resize(dsp_path.size());
  for(unsigned long int i = 0; i < dsp_path.size(); i++)
  {
    if(isSplined){
        msg.poses[i].pose.position.x = dsp_path[i][0] * res_octomap + pmin(0);
        msg.poses[i].pose.position.y = dsp_path[i][1] * res_octomap + pmin(1);
        msg.poses[i].pose.position.z = dsp_path[i][2] * res_octomap + pmin(2);
    }
    else{
        msg.poses[i].pose.position.x = dsp_path[i][0] + pmin(0);
        msg.poses[i].pose.position.y = dsp_path[i][1] + pmin(1);
        msg.poses[i].pose.position.z = dsp_path[i][2] + pmin(2);
    }
  }

  return msg; 
}


// publishing map usedfule when debuging
void Dsp::publishOccupancyGrid()
{
    visualization_msgs::msg::Marker occmap_viz;

    std::vector<geometry_msgs::msg::Point> marker_pos;
    std::vector<geometry_msgs::msg::Point> risk_pos;
     
    for(double x = 0; x < length_voxel; x++)
    {

        for(double y = 0; y < width_voxel; y++)
        {
            for(double z = 0; z < height_voxel; z++)
            {
                Eigen::Vector3d pos(x, y, z);
                //int idx = x + y*length + z*length*width;

                // Different parts to vizulize
                if(gdsl_->GetCost(pos) == DSP_OCCUPIED)
                //if(gdsl_->GetCost(pos) == DSP_UNKNOWN)
                //if(gdsl_->GetCost(pos) < 10)
                //if(gdsl_->GetCost(pos) == 1)
                //if(gdsl_->GetCost(pos) > 1 and gdsl_->GetCost(pos) < DSP_UNKNOWN)
                //if(gdsl_->GetCost(pos) < DSP_OCCUPIED and gdsl_->GetCost(pos) > DSP_UNKNOWN)
                //if(gdsl_->GetCost(pos) >= DSP_UNKNOWN / 5 and gdsl_->GetCost(pos) < DSP_UNKNOWN)
                {
                    //std::cout<<gdsl_->GetCost(pos)<<std::endl;
                    geometry_msgs::msg::Point pt;
                    //pt.x = (x + pmin(0)) * res_octomap;
                    //pt.y = (y + pmin(1)) * res_octomap;
                    //pt.z = (z + pmin(2)) * res_octomap;
                    pt.x = (x * res_octomap) + pmin(0);
                    pt.y = (y * res_octomap) + pmin(1);
                    pt.z = (z * res_octomap) + pmin(2);
                    marker_pos.push_back(pt);
                }  
            }  
        }
    }

    occmap_viz.header.frame_id = this->get_parameter("odom_frame_id").get_parameter_value().get<std::string>(); //odom_frame_id_; ///world /pixy/velodyne
    occmap_viz.header.stamp = rclcpp::Time();
    occmap_viz.ns = "dsp";
    occmap_viz.id = 1;
    occmap_viz.type = visualization_msgs::msg::Marker::CUBE_LIST;
    occmap_viz.action = visualization_msgs::msg::Marker::ADD;
    occmap_viz.pose.position.x = 0.5 * res_octomap;// + pmin(0) * res_octomap; 
    occmap_viz.pose.position.y = 0.5 * res_octomap;// + pmin(1) * res_octomap;
    occmap_viz.pose.position.z = 0.5 * res_octomap;// + pmin(2) * res_octomap;
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
    occ_map_viz_pub_->publish(occmap_viz);

}

} // namespace

int main (int argc, char * argv[])
{
  rclcpp::init (argc, argv);
  rclcpp::spin( std::make_shared<dsp::Dsp>() );
  rclcpp::shutdown();


  return 0;
}

