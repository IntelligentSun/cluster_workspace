/*
 *  service_client.h
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#ifndef SERVICE_SERVER_CPP
#define SERVICE_SERVER_CPP

#include "yikun_common/service/header.h"

namespace yikun_common {

class ServiceClient
{
public:
  typedef boost::shared_ptr<yikun_common::ServiceClient> Ptr;
  ServiceClient(const std::string addr,const int port);
  ~ServiceClient();
  /**
   * @brief 初始化 
   */
  bool initialize();
  /**
   * @brief 销毁 
   */
  bool destroy();
  /**
   * @brief 请求服务，移动到目标点
   * @param request 请求
   * @param response 响应
   */
  bool call(const _PoseStamped &request,_PoseStamped &response);
  bool callNoResponse(const _PoseStamped &request);
  /**
   * @brief 请求服务，设置导航路径
   * @param request 请求
   * @param response 响应
   */  
  bool call(const _Paths &request,_Paths &response);
  bool callNoResponse(const _Paths &request);

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

#endif // SERVICE_SERVER_CPP
