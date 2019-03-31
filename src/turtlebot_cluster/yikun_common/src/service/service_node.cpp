/*
 *  service_node.cpp
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include "yikun_common/service/service.h"

using namespace yikun_common;

int main(int argc,char* argv[])
{
  ros::init(argc,argv,"service_node");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh;
  Service service(local_nh);
  service.start();
  ros::spin();
  return 0;
}
