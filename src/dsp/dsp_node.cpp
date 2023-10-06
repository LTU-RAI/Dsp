#include "dsp/dsp.h"


int main (int argc, char **argv)
{
  ros::init (argc, argv, "dsp");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  dsp::Dsp dsp(nh, nh_private);

    //ros::MultiThreadedSpinner spinner(0);
    //spinner.spin();
    //ros::AsyncSpinner spinner(0);
    //spinner.start();
    //ros::waitForShutdown();
  ros::spin();


  return 0;
}


