/*
 *  robot_thread.cpp
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include "yikun_common/cluster_manager/robot_thread.h"

namespace yikun_common {

RobotThread::RobotThread(Robot robot)
  : robot_(robot),started_(false)
{}
RobotThread::~RobotThread()
{
  delete thread_;
  thread_ = NULL;
}

void RobotThread::start()
{
  nh_ = new ros::NodeHandle("~");
  thread_ = new boost::thread(boost::bind(&RobotThread::run,this));
  std::cout<<"robot_info thread create with id:"<<thread_->get_id()<<std::endl;
  started_ = true;
}

void RobotThread::stop()
{
  if(!started_) {
    return;
  }
  boost::unique_lock<boost::recursive_mutex> lock(stop_mutex_);
  nh_->shutdown();
  //线程回收
  std::cout<<"robot_info Thread interrupt with id:"<<thread_->get_id()<<std::endl;
  thread_->interrupt();
  thread_->join();
  delete thread_;
  thread_ = NULL;
  started_ = false;
  lock.unlock();
}

void RobotThread::run()
{
  boost::this_thread::interruption_point();

  //mysql
  //主机数据库连接
  local_db_ = boost::make_shared<DbHelper>("127.0.0.1","yk");
  while(!local_db_->dbconnect()&&nh_->ok()){
    ROS_ERROR("%s: Unable to connect to local db of %s",__func__,robot_.hostname.data());
    ros::Duration(1).sleep();
  }
  //对应小车数据库连接
  info_ = boost::make_shared<RobotInfo>(robot_);
  while(!info_->connect()&&nh_->ok()) {
    ROS_ERROR("%s: Unable to connect to target db of %s",__func__,robot_.hostname.data());
    ros::Duration(1).sleep();
  }

  while(nh_->ok())
  {
    float time = info_->checkCommunication();
    if(time == -1)
    {
      if(!info_->connect()) {
        ROS_ERROR("%s lost communication",robot_.hostname.data());
        continue;
      }
    } else {
      this->update();
    }
    ros::Duration(1.0).sleep();//更新频率（1/T）
  }
  //end
  boost::this_thread::interruption_enabled();
}

void RobotThread::update()
{
  RobotsInfo robots_info;
  robots_info.robot_id = robot_.id;
  robots_info.timestamp = ros::Time::now().toSec();
  RuntimeInfo info;
  if(!info_->getRuntimeInfo(info)) {
    ROS_ERROR("Could not get information");
    return;
  }
  robots_info.info = info;
  if(!local_db_->updateRobotsInfo(robots_info)) {
    ROS_ERROR("update infomation failed");
    return;
  }
  return;
}

}//namespace
