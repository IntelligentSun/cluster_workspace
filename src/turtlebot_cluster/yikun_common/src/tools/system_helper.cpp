/*
 *  system_helper.cpp
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include "yikun_common/tools/system_helper.h"

namespace yikun_common {

SystemHelper::SystemHelper()
{

}

bool SystemHelper::execute_shell(std::string cmd,std::string& out)
{
  FILE * stream;
  char buffer[MAXOUTPUTSIZE];
  cmd.append(" 2>&1");
  //
  stream = popen(cmd.c_str(), "r");
  if (stream) {
      while (!feof(stream))
      if (fgets(buffer, sizeof(buffer), stream) != NULL)
          out.append(buffer);
      pclose(stream);
      return true;
  }
  return false;
}

bool SystemHelper::getIpv4Address(std::string &addr,std::string device)
{
  struct ifaddrs *ifAddr=NULL,*ifa=NULL;

  if (getifaddrs(&ifAddr) == -1) {
    perror("getifaddrs");
    return false;
  }

  for (ifa = ifAddr; ifa != NULL; ifa = ifa->ifa_next) {
    if (ifa->ifa_addr == NULL)
      continue;
    if(device == ifa->ifa_name)
    {
      //ipv4
      if(ifa->ifa_addr->sa_family == AF_INET)
      {
        char host[NI_MAXHOST];
        int s = getnameinfo(ifa->ifa_addr,sizeof(struct sockaddr_in),
                        host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
        if (s != 0) {
            printf("getnameinfo() failed: %s\n", gai_strerror(s));
            break;
        }
        addr = host;
        freeifaddrs(ifAddr);
        ifAddr=NULL;
        return true;
      }
    }
  }

  freeifaddrs(ifAddr);
  ifAddr=NULL;
  return false;
}

float SystemHelper::ping(const std::string &ip, std::string &out)
{
  std::string cmd = "ping -c 1 ";
  cmd.append(ip);
  SystemHelper::execute_shell(cmd,out);
  if(out.find("rtt") == std::string::npos)
  {
    return -1;
  }
  std::vector<std::string> data;
  boost::split(data,out,boost::is_any_of("/"));

  return atof(data.at(4).c_str());
}

}//
