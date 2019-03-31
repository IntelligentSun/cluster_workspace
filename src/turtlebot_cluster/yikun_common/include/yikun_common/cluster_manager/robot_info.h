/*
 *  robot_info.h
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#ifndef ROBOT_INFO_H
#define ROBOT_INFO_H

#include "yikun_common/mysql/db_helper.h"
#include "yikun_common/tools/system_helper.h"

namespace yikun_common {

class RobotInfo
{
public:
  typedef boost::shared_ptr<yikun_common::RobotInfo> Ptr;
  RobotInfo(const Robot robot);
  ~RobotInfo();
	
  /**
   * @brief 从机数据库连接 
   */
  bool connect();
  /**
   * @brief 断开连接 
   */
  bool disconnect();
  /**
   * @brief 检查上位机与从机的通讯情况
   * @return 往返时延
   */
  float checkCommunication();
  /**
   * @brief 获取从机的状态信息
   * @param info 状态信息
   */
  bool getRuntimeInfo(RuntimeInfo &info);

private:
  Robot robot_;
  DbHelper::Ptr db_,cluster_db_;

};

	
}//namespace

#endif // ROBOT_INFO_H
