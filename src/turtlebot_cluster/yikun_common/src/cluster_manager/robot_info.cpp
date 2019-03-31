/*
 *  robot_info.cpp
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include "yikun_common/cluster_manager/robot_info.h"

namespace yikun_common {

RobotInfo::RobotInfo(const Robot robot)
  : robot_(robot)
{
  db_ = boost::make_shared<DbHelper>(robot_.addr,"yk");
  cluster_db_ = boost::make_shared<DbHelper>("127.0.0.1","yk");
}

RobotInfo::~RobotInfo()
{}

bool RobotInfo::connect()
{
  if(cluster_db_->dbconnect() && db_->dbconnect()) {
    return true;
  }
  return false;
}

bool RobotInfo::disconnect()
{
  if(cluster_db_->dbdisconnect() && db_->dbdisconnect()) {
    return true;
  }
  return false;
}

float RobotInfo::checkCommunication()
{
  if(db_.use_count() == 0) {
    ROS_ERROR("db has not initialized!!!");
    return -1;
  }
  std::string out;
  std::cout<<robot_.addr<<std::endl;
  float time_delay = SystemHelper::ping(robot_.addr,out);
//  std::cout<<robot_.addr<<std::endl;
  if(cluster_db_->updateTimeDelay(robot_.id,time_delay)) {
//    ROS_INFO("%s: %fms",robot_.hostname.data(),time_delay);
  }
  return time_delay;
}
bool RobotInfo::getRuntimeInfo(RuntimeInfo &info)
{
  if(db_.use_count() == 0) {
    ROS_ERROR("db has not initialized!!!");
    return false;
  }
  if(db_->getRuntimeInfo(info)) {
//    int id = atoi(info.map_id.c_str());
//    info.map_id.erase();
//    if(db_->getMapStrId(info.map_id,id)) {
      return true;
//    }
  }
  return false;
}


}//namespace
