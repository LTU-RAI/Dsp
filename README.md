# DSP: D*+ path planner on a uniformly spaced 3D or 2D grid integrated with Octomap and nav_msgs/OccupancyGrid

This is an implementation of D*+ on a uniformly spaced 3D or 2D grid for use in global path planning. The user can specify start and goal positions by publishing to the relevant topics, and the generated paths will be published by the node.

This branch is for ROS 2, for ROS 1 see brand ROS 1.

# 1 Installation

Install the dsl library
```
    cd 
    git clone https://github.com/jhu-asco/dsl.git
    cd dsl
    git checkout 61cf588668309e87de209cd95f03a0f792a16c33
    mkdir build
    cd build
    cmake ..
    sudo make install
```

In your ROS workspace, clone the repository:
```
    git clone https://github.com/LTU-RAI/Dsp.git
```

Build Dsp `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release` from your workspace. It is recomended to build in Relese mode.

## Quickstart / Minimal Setup
To run DSP, `edit dsp_launch.py` so parameters:

* `map_topic` aligns with the topic the map is published on.
* `use_3d` to `True` if you are using an octomap (3d map, `False` for 2D).
* `odom_trame_id` is the `TF` frame you have the odometry in.
* `base_link_frame_id` is the `base_link` frame for the robot.
  
The run:
```
ros2 launch dsp dsp_launch.py
```

and publish the goal pose to `/dsp/set_goal`.
Note that the pose must be within the map.

### Pre requierments
You need to publish a map of either `octomap_msg::msg::Octomap` (3D) or `nav_msgs::msg::OccupancyGrid` (2D). You also need to publish the robots `TF`.

### Post requirements
DSP publishes the path as a `nav_msgs::msg::Path` msg, default topic `dsp/path`, and a splined version on `dsp/spline_path`. To follow this, you need a path following node that takes the path and publishes the next waypoint in the path to the controller (that you also need).

## Topics
### Subscribed
* `/dsp/set_start`: [geometry_msgs::msg::Point] Used to set the start position. (redundant)
* `/dsp/set_start`: [geometry_msgs::msg::Point] Used to set the goal position.
* `/octomap_full`: [octomap_msgs::msg::Octomap] Use for 3D map.
* `/map`: [nav_msgs::msg::OccupancyGrid] Used for 2D map.

### Published 
* `/dsp/occupancy_map`: [visualization_msgs::msg::Marker] A marker for the occupancy grid to be displayed in Rviz used for debugging. Can display different occupancy statuses of voxels in the map by changing the publisher in `dsp.cpp`.
* `/dsp/path`: [nav_msgs::msg::Path] The generated path from start to goal.
* `/dsp/spline_path`: [nav_msgs::msg::Path] A smooth path with extra waypoints between existing waypoints to avoid sharp angles in the path.


## Parameters
The user must specify either a mesh to load or the size of the occupancy grid.  If both are given, the mesh will be loaded into an occupancy grid of the size given.

* `map_topic`: [String] The topic of the input map.
* `lower_thresh`: [int] Limit for free space in 2D map.
* `upper_thresh`: [int] Limit for occupied space in 2D map.
* `risk`: [int] the amount of voxels next to occupied spaced that is risk area.
* `RATE`: [double] the delay in ms between the path is updated.
* `use_gazebo_odom`: [bool] True if using odom to start planing from. (redundant)
* `use_3d`: [bool] True if using 3D planning.
* `odom_topic`: [string] Topic for odom.
* `odom_frame_id`: [string] Frame to plan in.
* `unknown_value`: [int] Traversal const for unknown voxels.

# Paper
Thank you for citing the [$D^*_{+}$ paper](https://arxiv.org/abs/2112.05563),if you use the $D^*_{+}$ paht planner in your work
```
@article{KARLSSON2023119408,
title = {D+∗: A risk aware platform agnostic heterogeneous path planner},
journal = {Expert Systems with Applications},
volume = {215},
pages = {119408},
year = {2023},
issn = {0957-4174},
doi = {https://doi.org/10.1016/j.eswa.2022.119408},
url = {https://www.sciencedirect.com/science/article/pii/S0957417422024277},
author = {Samuel Karlsson and Anton Koval and Christoforos Kanellakis and George Nikolakopoulos},
keywords = {DSP, Path planing, Risk aware, Platform agnostic},
abstract = {This article establishes the novel D+∗, a risk-aware and platform-agnostic heterogeneous global path planner for robotic navigation in complex environments. The proposed planner addresses a fundamental bottleneck of occupancy-based path planners related to their dependency on accurate and dense maps. More specifically, their performance is highly affected by poorly reconstructed or sparse areas (e.g. holes in the walls or ceilings) leading to faulty generated paths outside the physical boundaries of the 3-dimensional space. As it will be presented, D+∗ addresses this challenge with three novel contributions, integrated into one solution, namely: (a) the proximity risk, (b) the modeling of the unknown space, and (c) the map updates. By adding a risk layer to spaces that are closer to the occupied ones, some holes are filled, and thus the problematic short-cutting through them to the final goal is prevented. The novel established D+∗ also provides safety marginals to the walls and other obstacles, a property that results in paths that do not cut the corners that could potentially disrupt the platform operation. D+∗ has also the capability to model the unknown space as risk-free areas that should keep the paths inside, e.g in a tunnel environment, and thus heavily reducing the risk of larger shortcuts through openings in the walls. D+∗ is also introducing a dynamic map handling capability that continuously updates with the latest information acquired during the map building process, allowing the planner to use constant map growth and resolve cases of planning over outdated sparser map reconstructions. The proposed path planner is also capable to plan 2D and 3D paths by only changing the input map to a 2D or 3D map and it is independent of the dynamics of the robotic platform. The efficiency of the proposed scheme is experimentally evaluated in multiple real-life experiments where D+∗ is producing successfully proper planned paths, either in 2D in the use case of the Boston dynamics Spot robot or 3D paths in the case of an unmanned areal vehicle in varying and challenging scenarios.}
}
```
