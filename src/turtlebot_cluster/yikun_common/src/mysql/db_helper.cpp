/*
 *  db_helper.cpp
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include "yikun_common/mysql/db_helper.h"
namespace yikun_common {

DbHelper::DbHelper(std::string addr,std::string dbname)
    : initialized_(false),
      dbaddr_(addr),dbname_(dbname)
{
    dbuser_ = "root";
    dbpasswd_ = "123";
    sql_ = mysql_init((MYSQL*)0);
}

bool DbHelper::dbconnect()
{
  if(1 == sql_->reconnect)
  {
    return true;
  }
  if(sql_ != NULL && mysql_real_connect(sql_,dbaddr_.data(),dbuser_.data(),dbpasswd_.data(),dbname_.data(),3306,NULL,0))
  {
    if(!mysql_select_db(sql_,dbname_.data()))
    {
      ROS_DEBUG("select successfully.");
      sql_->reconnect = 1;
      initialized_ = true;
    }
  }
  else
  {
    ROS_DEBUG("Uable to connect the databases: %s,%s",dbaddr_.data(),dbname_.data());
  }
  return initialized_;
}
bool DbHelper::dbdisconnect()
{
  if(sql_->reconnect == 0){
    initialized_ = false;
    return true;
  }
  mysql_close(sql_);
  initialized_ = false;
  return true;
}

bool DbHelper::getRobots(std::vector<Robot> &robots)
{
  MYSQL_RES *res;
  MYSQL_ROW row;
  char tmp[NORMALSIZE];
  std::sprintf(tmp,"SELECT `id`,`host_name`,`ip`,`port`,`time_delay` FROM `robots`;");
  int ret = mysql_real_query(sql_,tmp,strlen(tmp));
  if(ret)
  {
    ROS_ERROR("%s: %s\n",__func__,mysql_error(sql_));
    return false;
  }
  res = mysql_store_result(sql_);
  while(row = mysql_fetch_row(res)){
    Robot robot;
    robot.id = atoi(row[0]);
    robot.hostname = row[1];
    robot.addr = row[2];
    robot.port = row[3];
    robot.time_delay = atof(row[4]);
    robots.push_back(robot);
  }
  mysql_free_result(res);
  return true;
}

bool DbHelper::updateTimeDelay(const int id,const float delay)
{
  //
  char tmp[NORMALSIZE];
  std::sprintf(tmp,"UPDATE `robots` SET `time_delay` = '%f' WHERE `id` = %d;",
               delay,id);
  int ret = mysql_real_query(sql_,tmp,strlen(tmp));
  if(ret)
  {
    ROS_ERROR("%s: %s\n",__func__,mysql_error(sql_));
    return false;
  }
  else
  {
    return true;
  }
}
bool DbHelper::updateRobotsInfo(RobotsInfo& info)
{
  char tmp[NORMALSIZE];
  std::sprintf(tmp,"UPDATE `robots_info` SET `state`=%d,`pos_x`=%f,`pos_y`=%f,"
                   "`theta`=%f,`vel_x`=%f,`vel_th`=%f,`map_id`=\"%s\",`timestamp`=%f "
                   "WHERE `robot_id`=%d;",
               info.info.state,info.info.pos_x,info.info.pos_y,info.info.theta,
               info.info.vel_x,info.info.vel_th,
               info.info.map_id.data(),info.timestamp,info.robot_id);
  int ret = mysql_real_query(sql_,tmp,strlen(tmp));
  if(ret)
  {
    ROS_ERROR("%s: %s\n",__func__,mysql_error(sql_));
    return false;
  }
  else
  {
    return true;
  }
}

bool DbHelper::getRuntimeInfo(RuntimeInfo &info)
{
  MYSQL_RES *res;
  MYSQL_ROW row;
  char tmp[NORMALSIZE];
  std::sprintf(tmp,"SELECT `state`,`pos_x`,`pos_y`,`theta`,`battery`,`vel_x`,`vel_th`,`map_id` FROM `runtime_info` WHERE 1");
  int ret = mysql_real_query(sql_,tmp,strlen(tmp));
  if(ret)
  {
    ROS_ERROR("%s: %s\n",__func__,mysql_error(sql_));
    return false;
  }
  res = mysql_store_result(sql_);
  while(row = mysql_fetch_row(res)){
    info.state = atoi(row[0]);
    info.pos_x = atof(row[1]);
    info.pos_y = atof(row[2]);
    info.theta = atof(row[3]);
    info.battery = atoi(row[4]);
    info.vel_x = atof(row[5]);
    info.vel_th = atof(row[6]);
    info.map_id = row[7];
    mysql_free_result(res);
    return true;
  }
  mysql_free_result(res);
  return false;
}
bool DbHelper::updatePosToRuntimeInfo(const float x,const float y,const float theta)
{
  char tmp[NORMALSIZE];
  std::sprintf(tmp,"UPDATE `runtime_info` SET `pos_x`=%f,`pos_y`=%f,`theta`=%f WHERE 1;",
               x,y,theta);
  int ret = mysql_real_query(sql_,tmp,strlen(tmp));
  if(ret)
  {
    ROS_ERROR("%s: %s\n",__func__,mysql_error(sql_));
    return false;
  }
  else
  {
    return true;
  }
}

bool DbHelper::getMapStrId(std::string &map_id,const int id)
{
  MYSQL_RES *res;
  MYSQL_ROW row;
  char tmp[NORMALSIZE];
  std::sprintf(tmp,"SELECT `uuid` FROM `map` WHERE `id`=%d;",id);
  int ret = mysql_real_query(sql_,tmp,strlen(tmp));
  if(ret)
  {
    ROS_ERROR("%s: %s\n",__func__,mysql_error(sql_));
    return false;
  }
  res = mysql_store_result(sql_);
  while(row = mysql_fetch_row(res)){
    map_id = row[0];
    mysql_free_result(res);
    return true;
  }
  mysql_free_result(res);
  return false;
}

/*
bool DbHelper::updateRobotsList(std::string addr,std::string host_name)
{
  double current_time = ros::Time::now().toSec();
  char tmp[NORMALSIZE];
  std::sprintf(tmp,"update robots_list set addr=\"%s\",timestamp=%f where host_name=\"%s\";",
               addr.c_str(),current_time,host_name.c_str());
  int ret = mysql_real_query(sql_,tmp,strlen(tmp));
  if(ret)
  {
    ROS_ERROR("%s: %s\n",__func__,mysql_error(sql_));
    return false;
  }
  else
  {
    return true;
  }
}

bool DbHelper::selectHostsFromRobotsList(std::string host_name)
{
  MYSQL_RES *res;
  MYSQL_ROW row;
  char tmp[NORMALSIZE];
  std::sprintf(tmp,"select id from robots_list where host_name=\"%s\";",host_name.c_str());
  int ret = mysql_real_query(sql_,tmp,strlen(tmp));
  if(ret)
  {
    ROS_ERROR("%s: %s\n",__func__,mysql_error(sql_));
    return false;
  }
  res = mysql_store_result(sql_);
  while(row = mysql_fetch_row(res)){
    mysql_free_result(res);
    return true;
  }
  mysql_free_result(res);
  return false;
}

bool DbHelper::insertRobotsList(std::string addr, std::string host_name)
{
  double current_time = ros::Time::now().toSec();
  char tmp[NORMALSIZE];
  std::sprintf(tmp,"INSERT INTO robots_list(addr,host_name,timestamp) VALUES(\"%s\",\"%s\",%f)",
               addr.c_str(),host_name.c_str(),current_time);
  int ret = mysql_real_query(sql_,tmp,strlen(tmp));
  if(ret)
  {
    ROS_ERROR("%s: %s\n",__func__,mysql_error(sql_));
    return false;
  }
  return true;
}
*/
}//namespace
