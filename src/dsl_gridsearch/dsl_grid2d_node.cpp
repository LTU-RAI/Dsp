#include "dsl_gridsearch/dsl_grid2d.h"


int main (int argc, char **argv)
{
  ros::init (argc, argv, "dsl_grid2d");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  dsl_gridsearch::DslGrid2D dslg2d(nh, nh_private);

  ros::spin();

  return 0;
}


