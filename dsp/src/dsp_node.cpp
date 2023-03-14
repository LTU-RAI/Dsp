#include "dsp.hpp"


int main (int argc, char * argv[])
{
  rclcpp::init (argc, argv);
  rclcpp::spin( std::make_shared<dsp::Dsp>() );
  rclcpp::shutdown();


  return 0;
}


