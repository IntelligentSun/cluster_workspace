/*
 *  publisher.cpp
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include "yikun_common/service/publisher.h"

namespace yikun_common {

Publisher::Publisher(const std::string addr,const int port)
  : addr_(addr),port_(port),isconnected_(false)
{
  timeout_.tv_sec = 0.5;
  timeout_.tv_usec = 0;
}
Publisher::~Publisher()
{
  if(isconnected_) {
    close(socket_);
    socket_ = -1;
    isconnected_ = false;
  }
}

bool Publisher::initialize()
{
  socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);//PF_INET
  if(socket_ == -1) {
    return false;
  }
  //timeout
  if(setsockopt(socket_,SOL_SOCKET,SO_SNDTIMEO,(const char*)&timeout_,sizeof(timeout_)) == -1)
    return false;
  if(setsockopt(socket_,SOL_SOCKET,SO_RCVTIMEO,(const char*)&timeout_,sizeof(timeout_)) == -1)
    return false;

  memset(&serv_addr_, 0, sizeof(serv_addr_));
  serv_addr_.sin_family = AF_INET;
  serv_addr_.sin_addr.s_addr = inet_addr(addr_.data());
  serv_addr_.sin_port = htons(port_);

  if(connect(socket_, (struct sockaddr*) &serv_addr_, sizeof(serv_addr_))==-1){
    isconnected_ = false;
    return false;
  }
  else{
    isconnected_ = true;
    return true;
  }
}

bool Publisher::destroy()
{
  if(!isconnected_)
    return false;
  close(socket_);
  socket_ = -1;
  isconnected_ = false;
  return true;
}
void Publisher::publish(const _twist &cmd_vel)
{
  int len = sizeof(_twist);
  send(socket_,&cmd_vel,len,0);
}

void Publisher::publish(const _PoseWithCovarianceStamped &pose)
{
  int len = sizeof(_PoseWithCovarianceStamped);
  send(socket_,&pose,len,0);
}
void Publisher::publish(const _Paths &path)
{
  int len = sizeof(_Paths);
  send(socket_,&path,len,0);
}
}//namespace
