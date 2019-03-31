/*
 *  uwb_node.cpp
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include "yikun_common/uwb/uwb.h"
#include "yikun_cluster_msgs/UWB.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "yikun_common/service/publisher.h"

using namespace yikun_common;
std::string global_frame_id_ = "map";
//<tag id,turtlebot publisher>
std::map<int,Publisher::Ptr> pubs_;
int main(int argc,char* argv[])
{
  ros::init(argc,argv,"uwb_demo");
  ros::NodeHandle nh,private_nh;
  std::string dev;
  //基站位置初始化
  AnchorArray anchorArray;
  anchorArray.A0 = Eigen::Vector3f(0,     0,    2);
  anchorArray.A1 = Eigen::Vector3f(0,     5.42, 2);
  anchorArray.A2 = Eigen::Vector3f(7.83,  5.42, 2);
  anchorArray.A3 = Eigen::Vector3f(7.83,  0,    2.5);
  private_nh.param("dev",dev,std::string("/dev/ttyACM0"));
  private_nh.param("global_frame",global_frame_id_,std::string("map"));
  //UWB接口初始化
  UWB uwb_interface(anchorArray,dev,115200,1000);
  if(!uwb_interface.connect()){
    ROS_ERROR_STREAM("Connect error");
    exit(1);
  }
  //UWB数据发布接口
  ros::Publisher ros_pub = nh.advertise<yikun_cluster_msgs::UWB>("/uwb_pose",5);
	
  //Turtlebot接口初始化
#if 0
#define TURTLEBOT1 "192.168.99.51"  //tag14
#define TURTLEBOT2 "192.168.99.52"  //tag0
#define TURTLEBOT3 "192.168.99.53"  //tag25
#define TURTLEBOT4 "192.168.99.54"  //tag7
#define TURTLEBOT5 "192.168.99.55"  //tag1
#endif
  Publisher::Ptr pub01 = boost::make_shared<Publisher>(TURTLEBOT1,UWB_PORT);
  if(pub01->initialize()) {
    ROS_DEBUG("%s push back",TURTLEBOT1);
    pubs_[14] = pub01;
  }
  Publisher::Ptr pub02 = boost::make_shared<Publisher>(TURTLEBOT2,UWB_PORT);
  if(pub02->initialize()) {
    ROS_DEBUG("%s push back",TURTLEBOT2);
    pubs_[0] = pub02;
  }
  Publisher::Ptr pub03 = boost::make_shared<Publisher>(TURTLEBOT3,UWB_PORT);
  if(pub03->initialize()) {
    ROS_DEBUG("%s push back",TURTLEBOT3);
    pubs_[25] = pub03;
  }
  Publisher::Ptr pub04 = boost::make_shared<Publisher>(TURTLEBOT4,UWB_PORT);
  if(pub04->initialize()) {
    ROS_DEBUG("%s push back",TURTLEBOT4);
    pubs_[7] = pub04;
  }
  Publisher::Ptr pub05 = boost::make_shared<Publisher>(TURTLEBOT5,UWB_PORT);
  if(pub05->initialize()) {
    ROS_DEBUG("%s push back",TURTLEBOT5);
    pubs_[1] = pub05;
  }

  ros::Rate loop(20); //执行频率 20hz
  while(nh.ok()) {
    std::vector<UWBInfo> infos;
    uwb_interface.getNewData(infos);
    for(UWBInfo info : infos) {
      _PoseWithCovarianceStamped pose;
      yikun_cluster_msgs::UWB uwb_pose;
      uwb_pose.tag = info.tag_id;
      uwb_pose.header.stamp = ros::Time::now();
      uwb_pose.header.frame_id = global_frame_id_;
      pose.stamp = ros::Time::now().toSec();
      strcpy(pose.frame_id,global_frame_id_.c_str());
      pose.position.x = uwb_pose.x = info.position.x;
      pose.position.y = uwb_pose.y = info.position.y;
      pose.position.z = uwb_pose.z = 0;//2d
      pose.orientation.w = 1;
      pose.orientation.x = 0;pose.orientation.y = 0;pose.orientation.z = 0;
      pose.covariance[6*0+0] = 0.25 * 0.25;//协方差
      pose.covariance[6*1+1] = 0.25 * 0.25;
      std::map<int,Publisher::Ptr>::iterator it;
      it = pubs_.find(info.tag_id);//查找对应的Turtlebot
      if(it != pubs_.end())
      {
        it->second->publish(pose);//发送到对应的Turtlebot
      }
      ros_pub.publish(uwb_pose);
    }
    loop.sleep();
  }
  //接口销毁
  for(std::map<int,Publisher::Ptr>::iterator itc=pubs_.begin();itc!=pubs_.end();itc++)
  {
    itc->second->destroy();
  }
  pubs_.clear();
  return 0;
}
