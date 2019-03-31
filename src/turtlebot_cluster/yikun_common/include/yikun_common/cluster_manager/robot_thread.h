/*
 *  robot_thread.h
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#ifndef ROBOT_THREAD_H
#define ROBOT_THREAD_H
#include "yikun_common/mysql/db_helper.h"
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include "yikun_common/cluster_manager/robot_info.h"

namespace yikun_common {

class RobotThread
{
public:
  typedef boost::shared_ptr<yikun_common::RobotThread> Ptr;
  RobotThread(const Robot robot);
  ~RobotThread();
  /**
   * @brief 开始线程 
   */
  void start();
  /**
   * @brief 结束线程 
   */
  void stop();

private:
  /**
   * @brief 执行线程 
   */
  void run();
  /**
   * @brief 更新从机的状态信息 
   */
  void update();

  bool started_;
  ros::NodeHandle *nh_;
  Robot robot_;
  boost::thread *thread_;
  boost::recursive_mutex stop_mutex_;
  DbHelper::Ptr local_db_;
  RobotInfo::Ptr info_;


};
}//namespace


#endif // ROBOT_THREAD_H
