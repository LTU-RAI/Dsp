# DSL GridSearch: D*-lite on a uniformly spaced 3D or 2D grid

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

# DSL Grid3D

## Quickstart / Minimal Setup

Unarchive the mesh file:

		cd data 
		tar -zxvf hackerman2.tar.gz

Launch the rviz viewer:

		roslaunch dsl_gridsearch rviz.launch

Launch the dsl_grid3d main ros node and load a mesh:

		roslaunch dsl_gridsearch dsl_grid3d_campus.launch

## Topics
### Subscribed
* `/dsl_grid3d/set_start`: [geometry_msgs::Point] Used to set the start position.
* `/dsl_grid3d/set_start`: [geometry_msgs::Point] Used to set the goal position.
* `/dsl_grid3d/set_occupied`: [geometry_msgs::Point] Used to set a position as occupied.
* `/dsl_grid3d/set_unoccupied`: [geometry_msgs::Point] Used to set a position as unoccupied.
* `/dsl_grid3d/set_mesh_occupied`: [shape_msgs::Mesh] Used to set cells intersecting with a mesh as occupied.

### Published 
* `/dsl_grid3d/occupancy_map`: [visualization_msgs::Marker] A marker for the occupancy grid to be displayed in Rviz.
* `/dsl_grid3d/mesh`: [visualization_msgs::Marker] A marker for the mesh to be displayed in Rviz.
* `/dsl_grid3d/path`: [nav_msgs::Path] The generated path from start to goal.
* `/dsl_grid3d/optpath`: [nav_msgs::Path] An optimized version of the path which removes unnecessary waypoints.
* `/dsl_grid3d/splinepath`: [nav_msgs::Path] A spline interpolation of the waypoints from the original path.
* `/dsl_grid3d/splineoptpath`: [nav_msgs::Path] A spline interpolation of the waypoints from the optimized path.


## Parameters
The user must specify either a mesh to load or the size of the occupancy grid.  If both are given, the mesh will be loaded into an occupancy grid of the size given.

* `mesh_filename`: [string] Absolute filepath of a .stl mesh to be loaded into an occupancy grid at runtime.
* `cells_per_meter`: [double] The resolution of the occupancy grid in cells/meter.
* `spline_path_maxvelocity`: [double] Determines the time scaling of the spline interpolated paths.
* `use_textured_mesh`: [bool] Indicates whether a .dae mesh file is availble to be used for visualization.  If mesh_filename is not the empty string, the program will look for a file with the same name with a .dae extension.  This file will be visualized in Rviz.  If set to false, the .stl file will be visualized. 
* `grid_length`: [int] Length of the grid. If less than or equal to 0, the grid will be the same size as the bounding box of the loaded mesh.
* `grid_width`: [int] Width of the grid. If less than or equal to 0, the grid will be the same size as the bounding box of the loaded mesh.
* `grid_height`: [int] Height of the grid. If less than or equal to 0, the grid will be the same size as the bounding box of the loaded mesh.
* `grid_xmin`: [int] The world frame x value corresponding to the first cell of the grid.
* `grid_ymin`: [int] The world frame y value corresponding to the first cell of the grid.
* `grid_zmin`: [int] The world frame z value corresponding to the first cell of the grid.

# DSL Grid2D

## Quickstart / Minimal Setup

Unarchive the mesh file:

		cd data 
		tar -zxvf hackerman2.tar.gz

Launch the rviz viewer:

		roslaunch dsl_gridsearch rviz2d.launch

Launch the dsl_grid2d main ros node and load a mesh:

		roslaunch dsl_gridsearch dsl_grid2d_campus.launch

## Topics
### Subscribed
* `/dsl_grid2d/set_start`: [geometry_msgs::Point] Used to set the start position.
* `/dsl_grid2d/set_start`: [geometry_msgs::Point] Used to set the goal position.
* `/dsl_grid2d/set_cost`: [geometry_msgs::Point] Used to set the height of a position.
* `/dsl_grid2d/set_mesh_cost`: [shape_msgs::Mesh] Used to set cell heights to correspond to a given mesh.

### Published 
* `/dsl_grid2d/trav_map`: [visualization_msgs::Marker] A marker for the occupancy grid to be displayed in Rviz.
* `/dsl_grid2d/mesh`: [visualization_msgs::Marker] A marker for the mesh to be displayed in Rviz.
* `/dsl_grid2d/path`: [nav_msgs::Path] The generated path from start to goal.
* `/dsl_grid2d/optpath`: [nav_msgs::Path] An optimized version of the path which removes unnecessary waypoints.


## Parameters
The user must specify either a mesh to load or the size of the occupancy grid.  If both are given, the mesh will be loaded into an occupancy grid of the size given.

* `mesh_filename`: [string] Absolute filepath of a .stl mesh to be loaded into an occupancy grid at runtime.
* `cells_per_meter`: [double] The resolution of the occupancy grid in cells/meter.
* `use_textured_mesh`: [bool] Indicates whether a .dae mesh file is availble to be used for visualization.  If mesh_filename is not the empty string, the program will look for a file with the same name with a .dae extension.  This file will be visualized in Rviz.  If set to false, the .stl file will be visualized. 
* `grid_length`: [int] Length of the grid. If less than or equal to 0, the grid will be the same size as the bounding box of the loaded mesh.
* `grid_width`: [int] Width of the grid. If less than or equal to 0, the grid will be the same size as the bounding box of the loaded mesh.
* `grid_xmin`: [int] The world frame x value corresponding to the first cell of the grid.
* `grid_ymin`: [int] The world frame y value corresponding to the first cell of the grid.

