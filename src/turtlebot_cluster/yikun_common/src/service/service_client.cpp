/*
 *  service_client.cpp
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include "yikun_common/service/service_client.h"

namespace yikun_common {

ServiceClient::ServiceClient(const std::string addr,const int port)
  : addr_(addr),port_(port),isconnected_(false)
{
  timeout_.tv_sec = 60;
  timeout_.tv_usec = 0;
}
ServiceClient::~ServiceClient()
{
  if(isconnected_) {
    close(socket_);
    socket_ = -1;
    isconnected_ = false;
  }
}

bool ServiceClient::initialize()
{
  socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);//PF_INET
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

bool ServiceClient::destroy()
{
  if(!isconnected_)
    return false;
  close(socket_);
  socket_ = -1;
  isconnected_ = false;
  return true;
}
bool ServiceClient::call(const _PoseStamped &request, _PoseStamped &response)
{
  int len = sizeof(_PoseStamped);
  if(send(socket_,&request,len,0) == -1 && errno == EAGAIN)
  {
    perror("Send");
    return false;
  }
  if(recv(socket_,&response,len,0) == -1 && errno == EAGAIN)
  {
    perror("Recv");
    return false;
  }
  return true;
}
bool ServiceClient::callNoResponse(const _PoseStamped &request)
{
  int len = sizeof(_PoseStamped);
  if(send(socket_,&request,len,0) == -1 && errno == EAGAIN)
  {
    perror("Send");
    return false;
  }
  return true;
}

bool ServiceClient::call(const _Paths &request, _Paths &response)
{
  int len = sizeof(_Paths);
  if(send(socket_,&request,len,0) == -1 && errno == EAGAIN)
  {
    perror("Send");
    return false;
  }
  if(recv(socket_,&response,len,0) == -1 && errno == EAGAIN)
  {
    perror("Recv");
    return false;
  }
  return true;
}

bool ServiceClient::callNoResponse(const _Paths &request)
{
  int len = sizeof(_Paths);
  if(send(socket_,&request,len,0) == -1 && errno == EAGAIN)
  {
    perror("Send");
    return false;
  }
  return true;
}

}//namespace
