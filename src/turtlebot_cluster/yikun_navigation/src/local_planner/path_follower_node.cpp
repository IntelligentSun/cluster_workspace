/*
 *  path_follower_node.cpp
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include "yikun_navigation/local_panner/path_follower.h"
#include <boost/thread.hpp>
#include <geometry_msgs/Twist.h>

using namespace yikun_navigation;
int main(int argc,char* argv[])
{
  ros::init(argc,argv,"path_follower");
  ros::NodeHandle nh;
  tf::TransformListener tf(ros::Duration(10));
  PathFollower::Ptr path_follower = boost::make_shared<PathFollower>();
  path_follower->initialize(&tf);
  ros::spin();
  return 0;
}
