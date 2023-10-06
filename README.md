# DSP: D*+ path planner on a uniformly spaced 3D or 2D grid integrated with Octomap and cartographer

This is an implementation of D*+ on a uniformly spaced 3D or 2D grid for use in global path planning. The user can specify start and goal positions by publishing to the relevant topics, and the generated paths will be published by the node.

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
We tested DSP on Ubuntu 18.04 and 20.04 thuse ROS melodic and noetic.


In your ROS package path, clone the repository:
```
    git clone https://github.com/LTU-RAI/Dsp.git
```

Build Dsp `catkin build` from your catkin work space.

## Quickstart / Minimal Setup
To use DSL with octomap in gazebo set `frame_id` and `cloud_in` to the map frame used and point-cloud you use in `launch/octomap_gazebo.launch`, make sure you have `tf` configured between the map frame and your odometry. And in dsp_grid3d_gazebo set `odom_topic` to your odometry. And finally, change the odometry topic and position reference topic to match your drone's odometry and controller in `src/dsp/path_to_pose.py`.
The run:
```
roslaunch dsp dsp_3d_start.launch
```
and publich the gole pose to `/dsp/set_goal`.
Observe that the pose has to be inside the map.


## Topics
### Subscribed
* `/dsp/set_start`: [geometry_msgs::Point] Used to set the start position.
* `/dsp/set_start`: [geometry_msgs::Point] Used to set the goal position.
* `/octomap_full`: [octomap_msgs::Octomap] Use for 3D map.
* `/map`: [nav_msgs::OccupancyGrid] Used for 2D map.

### Published 
* `/dsp/occupancy_map`: [visualization_msgs::Marker] A marker for the occupancy grid to be displayed in Rviz used for debugging. Can display different occupancy statuses of voxels in the map by changing the publisher in `dsp.cpp`.
* `/dsp/path`: [nav_msgs::Path] The generated path from start to goal.
* `/dsp/optpath`: [nav_msgs::Path] An optimized version of the path which removes unnecessary waypoint


## Parameters
The user must specify either a mesh to load or the size of the occupancy grid.  If both are given, the mesh will be loaded into an occupancy grid of the size given.

* `map_topic`: [String] The topic of the input map.
* `lower_thresh`: [int] Limit for free space in 2D map.
* `upper_thresh`: [int] Limit for occupied space in 2D map.
* `risk`: [int] the amount of voxels next to occupied spaced that is risk area.
* `use_gazebo_odom`: [bool] True if using odom to start planing from.
* `use_3d`: [bool] True if using 3D planning.
* `odom_topic`: [string] Topic for odom.
* `odom_frame_id`: [string] Frame to plan in.
* `unknown_value`: [int] Traversal const for unknown voxels.

# Paper
Thank you for citing the [$D^*_{+}$ paper](https://arxiv.org/abs/2112.05563),if you use the $D^*_{+}$ paht planner in your work
```
@article{karlsson2022d+,
  title={D+∗: A risk aware platform agnostic heterogeneous path planner},
  author={Karlsson, Samuel and Koval, Anton and Kanellakis, Christoforos and Nikolakopoulos, George},
  journal={Expert systems with applications},
  pages={119408},
  year={2022},
  publisher={Elsevier}
}

```
