# DSL GridSearch: D*-lite on a uniformly spaced 3D or 2D grid integrated with Octomap

This is an implementation of D*-lite graph search on a uniformly spaced 3D or 2D grid for use in global path planning.  This package provides the ability to create an occupancy grid from a .stl mesh or to specify a grid of a given size.  The user can specify start and goal positions by publishing to the relevant topics, and the generated paths will be published by the node.  The user can also publish messages to set grid cells to be occupied or unoccupied, or mesh messages can be sent to set all cells which intersect with the mesh as occupied.  The 2D version is implemented as a traversability map, where the cost of an edge is equal to the height gradient between two grid cells.

# 1 Installation
We tested DSL GridSearch on Ubuntu 12.04 (Precise) and ROS hydro.

In your ROS package path, clone the repository:

    git clone https://github.com/jhu-asco/dsl_gridsearch.git

Install the dsl library

    cd 
    git clone https://github.com/jhu-asco/dsl.git
    cd dsl
    git checkout 61cf588668309e87de209cd95f03a0f792a16c33
    mkdir build
    cd build
    cmake ..
    sudo make install

Run catkin_make from the workspace root directory, as usual.

If you get linker errors related to trimesh, e.g.

    undefined reference to trimesh::TriMesh::read(std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&)' collect2: error: ld returned 1 exit status dsl_gridsearch/CMakeFiles/dsl_grid3d_node.dir/build.make:120: recipe for target '/home/sam/robo/devel/lib/dsl_gridsearch/dsl_grid3d_node' failed make[2]: *** [/home/sam/robo/devel/lib/dsl_gridsearch/dsl_grid3d_node] Error 1 CMakeFiles/Makefile2:377: recipe for target 'dsl_gridsearch/CMakeFiles/dsl_grid3d_node.dir/all' failed

then build trimesh2 (https://github.com/Forceflow/trimesh2.git) from source and copy the bin and include directory to extern/trimesh (See https://github.com/jhu-asco/dsl_gridsearch/issues/3)

## Quickstart / Minimal Setup
To use DSL with octomap in gazebo set `frame_id` and `cloud_in` to the map frame used and point-cloud you use in `launch/octomap_gazebo.launch`, make sure you have `tf` configured between the map frame and your odometry. And in dsl_grid3d_gazebo set `odom_topic` to your odometry. And finally, change the odometry topic and position reference topic to match your drone's odometry and controller in `src/dsl_gridsearch/path_to_pose.py`.
The run:
```
rosrun dsl_gridsearch dsl_3d_start.launch
```
and publich the gole pose to `/dsl_grid3d/set_goal`.
Observe that the pose has to be inside the map.


## Topics
### Subscribed
* `/dsl_grid3d/set_start`: [geometry_msgs::Point] Used to set the start position.
* `/dsl_grid3d/set_start`: [geometry_msgs::Point] Used to set the goal position.
* `/octomap_full`: [octomap_msgs::Octomap] Use for 3D map.
* `/map`: [nav_msgs::OccupancyGrid] Used for 2D map.

### Published 
* `/dsl_grid3d/occupancy_map`: [visualization_msgs::Marker] A marker for the occupancy grid to be displayed in Rviz used for debugging. Can display different occupancy statuses of voxels in the map by changing the publisher in `dsl_grid3d.cpp`.
* `/dsl_grid3d/path`: [nav_msgs::Path] The generated path from start to goal.
* `/dsl_grid3d/optpath`: [nav_msgs::Path] An optimized version of the path which removes unnecessary waypoint


## Parameters
The user must specify either a mesh to load or the size of the occupancy grid.  If both are given, the mesh will be loaded into an occupancy grid of the size given.

* `map_topic`: [String] The topic of the input map.
* `lower_thresh`: [int] Limit for free space in 2D map.
* `upper_thresh`: [int] Limit for occupied space in 2D map.
* `risk`: [int] the amount of voxels next to occupied spaced that is risk area.
* `use_gaxebo_odom`: [bool] True if using odom to start planing from.
* `use_3d`: [bool] True if using 3D planning.
* `odom_topic`: [string] Topic for odom.
* `odom_frame_id`: [string] Frame to plan in.
* `unknown_value`: [int] Traversal const for unknown voxels.





## Issues

Error 1:

	REQUIRED PACKAGE NOT FOUND
	DSL_INCLUDE_DIR=/usr/include
	DSL_LIBRARY=<not found>

FIX: 

	cd 
	git clone https://github.com/jhu-asco/dsl.git
	cd dsl
	git checkout 61cf588668309e87de209cd95f03a0f792a16c33
	mkdir build
	cd build
	cmake ..
	sudo make install

Error 2: 

	Errors 	<< dsl_gridsearch:make /home/anton/ros_workspaces/temp_ws/logs/dsl_gridsearch/build.make.002.log
	/usr/bin/ld: /home/anton/ros_workspaces/temp_ws/src/dsl_gridsearch/extern/trimesh/bin/libtrimesh.a(TriMesh_grid.o): relocation R_X86_64_32 against `.rodata.str1.1' can not be used when making a PIE object; recompile with -fPIC

FIX:

	git clone https://github.com/Forceflow/trimesh2.git
	cd trimesh2/
	sudo make
	Then: copy libtirmesh.a from “trimesh2/lib.Linux64” to “dsl_gridsearch/extern/trimesh/bin”
	 And
	Copy “trimesh2/include” to “dsl_gridsearch/extern/trimesh/include/trimesh2”

Error 3:

	[dsl_grid3d-1] process has died 

FIX:

	cd data 
	tar -zxvf hackerman2.tar.gz

