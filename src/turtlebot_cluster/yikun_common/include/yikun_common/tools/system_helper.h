/*
 *  system_helper.h
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#ifndef SYSTEM_HELPER_H
#define SYSTEM_HELPER_H
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <vector>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <fstream>
#include <dirent.h>
#include <fcntl.h>

#include <ifaddrs.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>

#define MAXOUTPUTSIZE 1024

namespace yikun_common {

class SystemHelper
{
public:
  SystemHelper();
  /**
   * @brief 执行shell命令
   * @param command 命令
   * @param out     输出
   */
  static bool execute_shell(std::string command,std::string& out);
  /**
   * @brief 获取网络设备的ipv4地址
   * @param addr    设备地址
   * @param device  设备名
   * @return
   */
  static bool getIpv4Address(std::string& addr,std::string device);
  /**
   * @brief 测试当前设备与另一主机的往返时延
   * @param ip  ip地址
   * @param out 输出
   */
  static float ping(const std::string &ip, std::string &out);

private:

};

}//namespace

#endif // SYSTEM_HELPER_H
