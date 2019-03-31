/*
 *  db_helper.h
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#ifndef DB_HELPER_H
#define DB_HELPER_H
#include "mysql_head.h"

namespace yikun_common {

class DbHelper
{
public:
  typedef boost::shared_ptr<yikun_common::DbHelper> Ptr;
  DbHelper(std::string addr,std::string dbname);
  //interface
  bool dbconnect();
  bool dbdisconnect();
  /**
   * @brief function
   * */
  //robots
  /**
   * @brief 获取小车队列
   * @param robots 小车队列
   */
  bool getRobots(std::vector<Robot> &robots);
  /**
   * @brief 更新对应id的小车的往返时延
   * @param id 小车的id
   * @param delay 往返时延
   */
  bool updateTimeDelay(const int id,const float delay);
  //robots info
  /**
   * @brief 更新小车状态
   * @param info 小车状态 
   */
  bool updateRobotsInfo(RobotsInfo& info);
  //runtime info
  /**
   * @brief 获取小车状态
   * @param info 小车状态
   */
  bool getRuntimeInfo(RuntimeInfo &info);
  /**
   * @brief 更新小车位置 
   */
  bool updatePosToRuntimeInfo(const float x,const float y,const float theta);
  //map
  /**
   * @brief 获取对图id 
   */
  bool getMapStrId(std::string &map_id,const int id);

private:
  MYSQL *sql_;
  std::string dbuser_,dbaddr_,dbpasswd_,dbname_;
  bool initialized_;
};

}//namespace

#endif // DB_HELPER_H
