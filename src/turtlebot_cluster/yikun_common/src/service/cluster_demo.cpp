/*
 *  cluster_demo.cpp
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include "yikun_common/service/service_client.h"
#include "yikun_common/service/publisher.h"
#include <geometry_msgs/Twist.h>

using namespace yikun_common;
//速度监听
void callback(const geometry_msgs::Twist::ConstPtr& message);
std::vector<Publisher::Ptr> pubs_;//发布器队列

int main(int argc,char* argv[])
{
  ros::init(argc,argv,"cluster_demo");
  ros::NodeHandle nh;
  std::string turtlebot_01,turtlebot_02,turtlebot_03,turtlebot_04,turtlebot_05;
  ros::NodeHandle private_nh;
  //小车定义
  std::vector<std::string> addrs;
  private_nh.param("turtlebot_01",turtlebot_01,std::string("192.168.99.51"));
  private_nh.param("turtlebot_02",turtlebot_02,std::string("192.168.99.52"));
  private_nh.param("turtlebot_03",turtlebot_03,std::string("192.168.99.53"));
  private_nh.param("turtlebot_04",turtlebot_04,std::string("192.168.99.54"));
  private_nh.param("turtlebot_05",turtlebot_05,std::string("192.168.99.55"));
  addrs.push_back(turtlebot_01);
  addrs.push_back(turtlebot_02);
  addrs.push_back(turtlebot_03);
  addrs.push_back(turtlebot_04);
  addrs.push_back(turtlebot_05);
  //速度接口定义
  ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel",1000,callback);

  //移动到目标位置点测试
  /*
  while(nh.ok()) {
    ServiceClient::Ptr client = boost::make_shared<ServiceClient>("192.168.1.159",TCP_PORT);
    if(client->initialize()) {
      _PoseStamped pos,response;
      pos.ack = ACK_FAILED;
      strcpy(pos.frame_id,"map");
      pos.position.x = 0.1;
      pos.orientation.w = 0.2;
      pos.stamp = ros::Time::now().toSec();
      if(client->call(pos,response)) {
        if(response.ack == ACK_SUCCEED)
          ;
        else
          ROS_INFO("response %d,%.4f,%.4f",response.ack,response.stamp,response.orientation.w);
      } else {
        ROS_ERROR("call");
      }
    } else {
      ROS_ERROR("client initialize");
    }
    client->destroy();
    ros::Duration(0.5).sleep();  }
  */
	
  //发布器初始化
  for(auto addr : addrs) {
    Publisher::Ptr pub = boost::make_shared<Publisher>(addr,UDP_PORT);
    if(pub->initialize()) {
      ROS_INFO("%s push back",addr.data());
      pubs_.push_back(pub);
    }
  }
	
  //发布器测试
  /*
  if(pub->initialize()) {
    _twist twist;
//    for(int i=0;i<10;i++){
    while(ros::ok()){
      twist.linear_x = 0.0;
      twist.angular_z = 0.5;
      pub->publish(twist);
      ros::Duration(0.01).sleep();
    }
  } else {
    ROS_ERROR("pub initialize");
  }
  pub->destroy();
  */
  ros::spin();
  //发布器销毁
  for(Publisher::Ptr pub : pubs_) {
    pub->destroy();
  }
  return 0;
}

void callback(const geometry_msgs::Twist::ConstPtr &message)
{
  _twist twist;
  twist.linear_x = message->linear.x;	
  twist.angular_z = message->angular.z;
  for(Publisher::Ptr pub : pubs_) {			//发布到对应主机
    pub->publish(twist);
  }
}
