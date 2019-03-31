/*
 *    robot_info_sync.cpp
 *    Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */

#include <ros/ros.h>
#include "yikun_common/mysql/db_helper.h"
#include "yikun_common/cluster_manager/robot_thread.h"

using namespace yikun_common;

//global value
bool          isconnected_;//true: online;  false: offline
double        frequency_;
std::vector<RobotThread::Ptr> threadptr_;//小车线程队列

//global function
void sendtoMaster();

///main
int main(int argc,char *argv[])
{
  ros::init(argc,argv,"robot_info_sync");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.param("frequency",frequency_,(double)10);
  isconnected_ = false;
  DbHelper::Ptr db_;
  db_.reset();
  db_ = boost::make_shared<DbHelper>("127.0.0.1","yk");
  while(!db_->dbconnect()){
    ROS_ERROR("Unable to connect to mysql");
    sleep(1);
  }
//  ros::Rate loop(frequency_);
  std::cout<<"Cluster Config Ready"<<std::endl;
  if(nh.ok())
  {
    //小车队列
    std::vector<Robot> robots;
    if(!db_->getRobots(robots)) {
      ROS_ERROR("Could not get robots from db");
      exit(1);
    }
    for(Robot robot : robots)
    {
      //小车信息更新同步线程
      RobotThread::Ptr thread = boost::make_shared<RobotThread>(robot);
      thread->start();
      threadptr_.push_back(thread);
    }
  }
  ros::spin();
  return 0;
}
