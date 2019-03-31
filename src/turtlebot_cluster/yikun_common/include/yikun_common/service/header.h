/*
 *  header.h
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#ifndef HEADER_H
#define HEADER_H
#include <iostream>
#include <stdlib.h>
#include <string>
#include <cstring>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <nav_msgs/Path.h>

///define
//路径最大缓存大小
#define PATH_MAX_SIZE 4096
//Turtlebot IP地址声明
#define TURTLEBOT1 "192.168.99.51"  //tag14
#define TURTLEBOT2 "192.168.99.56"  //tag0
#define TURTLEBOT3 "192.168.99.53"  //tag25
#define TURTLEBOT4 "192.168.99.54"  //tag7
#define TURTLEBOT5 "192.168.99.55"  //tag1

//监听目标点端口
#ifndef TCP_PORT
#define TCP_PORT 25555
#endif
//监听速度端口
#ifndef UDP_PORT
#define UDP_PORT 26666
#endif
//监听UWB定位信息端口
#ifndef UWB_PORT
#define UWB_PORT 27777
#endif
//监听路径端口
#ifndef PATH_PORT
#define PATH_PORT 28888
#endif

namespace yikun_common {

typedef boost::array<double, 36> _covariance_type;
typedef std::basic_string<char, std::char_traits<char>> _frame_id_type;

//ACK
typedef enum {
  ACK_SUCCEED   = 1,
  ACK_FAILED    = 2
}_ack;
//service
typedef enum {
  SetPose = 1,
  SendTo  = 2,
  EStop   = 3,
  Cancel  = 4
}_service_type;
//
//位置点
typedef struct Position {
  float x;
  float y;
  float z;
}_position;
//姿态
typedef struct Orientation {
  float x;
  float y;
  float z;
  float w;
}_orientation;

//udp
//速度
typedef struct Twist {
  float linear_x;
  float angular_z;
}_twist;
//位置点
typedef struct PoseWithCovarianceStamped{
  double            stamp;          //时间戳
  char              frame_id[30];   //坐标
  _position         position;       //位置点
  _orientation      orientation;    //位置姿态
  double            covariance[36]; //协方差矩阵
}_PoseWithCovarianceStamped;
//位置点
typedef struct PoseStamped {
  unsigned int  ack;          //确认
  double        stamp;        //时间戳
  char          frame_id[30]; //坐标
  _position     position;     //位置点
  _orientation  orientation;  //姿态
}_PoseStamped;
//路径
typedef struct Paths {
  unsigned int  ack;                  //确认
  char          paths[PATH_MAX_SIZE]; //路径
}_Paths;



}//namespace

#endif // HEADER_H
