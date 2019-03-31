/*
 *  path_finder.cpp
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "yikun_common/service/service_client.h"
#include "yikun_common/service/publisher.h"

using namespace yikun_common;
/**
 * @brief 路径解析
 */
bool getPath(const char *p,std::vector<geometry_msgs::PoseStamped> &path);
/**
 * @brief 发布路径
 * @param path 路径
 * @param pub 发布器接口
 */
void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path,
                 const ros::Publisher& pub);

//global value
std::string global_frame_id_ = "map";

int main(int argc,char* argv[])
{
  ros::init(argc,argv,"path_finder");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  std::string file_path;
  //Initial robot list
  std::map<int,std::string> robots;
  robots[14]  = "192.168.99.51";  //tag  14
  robots[0]   = "192.168.99.56";   //tag  0
  robots[25]  = "192.168.99.53";  //tag  25
  robots[7]   = "192.168.99.54";   //tag  7
  robots[1]   = "192.168.99.55";   //tag  1
  //路径文件地址
  private_nh.param("file_path",file_path,std::string("path.yaml"));
  std::cout<<file_path<<std::endl;
  std::ifstream in(file_path);
  std::string data;
  for(std::string s;in>>s;)
  {
    data.append(s);
  }
  //路径解析
  std::vector<std::string> paths;
  if(data.find('-') == std::string::npos)
    return false;
  boost::split(paths,data,boost::is_any_of("-"));
  std::cout<<paths.size()<<std::endl;

  std::vector<ros::Publisher> pubs; //pub buff
  std::vector<std::string> plans;   //plan buff
  for(std::string pathstr : paths) {
//    std::cout<<pathstr<<std::endl;
    if(pathstr.find(':') == std::string::npos)
      continue;
    std::vector<std::string> tmp;
    boost::split(tmp,pathstr,boost::is_any_of(":"));
    int id = atoi(tmp.front().c_str());
    _Paths path;
    std::memset(&path,0,sizeof(_Paths));
    std::strcpy(path.paths,tmp.back().c_str());
    std::cout<<tmp.back().size()<<std::endl;
    //ros gui
    std::string topic = "/turtlebot";
    topic.append(tmp.front().c_str());
    topic.append("/global_path");
    ros::Publisher pub = nh.advertise<nav_msgs::Path>(topic.c_str(),1);
    pubs.push_back(pub);
    plans.push_back(path.paths);
//    publishPlan(gui_path,pub);
//    pub.shutdown();
    Publisher::Ptr my_pub = boost::make_shared<Publisher>(robots[id],PATH_PORT);
    if(my_pub->initialize()) {
      ROS_INFO("%s push back",robots[id].data());
my_pub->publish(path);
//      pubs_.push_back(pub);
    }
    
    //new service client
//    ServiceClient::Ptr client = boost::make_shared<ServiceClient>(robots[id],PATH_PORT);
//    if(!client->initialize()) {
//      ROS_ERROR("Initial Error");
//      continue;
//    }
//    //发送路径请求
//    if(client->callNoResponse(path)) {
//      ROS_INFO("request succeed");
//    }
    ros::Duration(3.5).sleep();//delay
  }
  exit(1);
  //gui
  while(nh.ok()) {
    int i = 0;
    for(ros::Publisher pub : pubs) {
      std::vector<geometry_msgs::PoseStamped> gui_path;
      if(!getPath(plans.at(i).c_str(),gui_path)) {
        ROS_ERROR("path error(%d)",i);
        exit(1);
      }
      publishPlan(gui_path,pub);
      i++;
    }
    ros::Duration(1).sleep();
  }
//  ros::spin();
  return 0;
}

bool getPath(const char *p,std::vector<geometry_msgs::PoseStamped> &path)
{
  path.clear();
//  ROS_WARN("%s",p);
  //路径解析，将路径转化为标准路径格式
  std::string pathstr = p;
  std::vector<std::string> points;
  if(pathstr.find(';') == std::string::npos)
    return false;
  boost::split(points,pathstr,boost::is_any_of(";"));
  for(std::string point : points)
  {
    std::vector<std::string> index;
    geometry_msgs::PoseStamped pose;
    if(point.find(',') == std::string::npos)
      return true;
    boost::split(index,point,boost::is_any_of(","));
    pose.header.frame_id = global_frame_id_;
    pose.header.stamp = ros::Time::now();
    //（x，y）根据实际路径文件赋值
    pose.pose.position.x = atof(index[2].c_str())*0.01;	//x
    pose.pose.position.y = atof(index[1].c_str())*0.01;	//y
    //姿态初始化，UWB数据中无姿态
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    path.push_back(pose);
  }
  return true;
}
void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub)
{
  //given an empty path we won't do anything
  if(path.empty())
    return;

  //create a path message
  nav_msgs::Path gui_path;
  gui_path.poses.resize(path.size());
  gui_path.header.frame_id = path[0].header.frame_id;
  gui_path.header.stamp = path[0].header.stamp;

  // Extract the plan in world co-ordinates, we assume the path is all in the same frame
  for(unsigned int i=0; i < path.size(); i++){
    gui_path.poses[i] = path[i];
  }
//  while(ros::ok()){
  pub.publish(gui_path);
  ros::Duration(0.5).sleep();
//  }
}
