#include "yikun_navigation/filter/ros_filter.h"
#include "yikun_navigation/filter/ekf.h"

using namespace RobotLocalization;

int main(int argc,char* argv[])
{
  ros::init(argc,argv,"ekf_node");
  RosFilter<Ekf> ekf;
  ekf.run();
  return 0;
}
