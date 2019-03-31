/*
 *  publisher.h
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#ifndef PUBLISHER_H
#define PUBLISHER_H

#include "yikun_common/service/header.h"

namespace yikun_common {

class Publisher
{
public:
  typedef boost::shared_ptr<yikun_common::Publisher> Ptr;
  Publisher(const std::string addr,const int port);
  ~Publisher();
  /**
   * @brief 初始化 
   */
  bool initialize();
  /**
   * @brief 销毁 
   */
  bool destroy();
  /**
   * @brief 发布速度
   * @param cmd_vel 速度
   */
  void publish(const _twist &cmd_vel);
  /**
   * @brief 发布位置
   * @param pose 位置 
   */
  void publish(const _PoseWithCovarianceStamped& pose);
  void publish(const _Paths& path);

private:
  std::string addr_;
  int port_;
  struct sockaddr_in serv_addr_;
  int socket_;
  bool isconnected_;
  int timesec_;
  struct timeval timeout_;
};

}//namespace

#endif // PUBLISHER_H
