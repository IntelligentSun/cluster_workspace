/*
 *  mysql_head.h
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#ifndef MYSQL_HEAD_H
#define MYSQL_HEAD_H
#include <iostream>
#include <ros/ros.h>
#include <mysql/mysql.h>
#include <string>
#include <vector>
#include <set>
#include <algorithm>
#include <map>
#include <time.h>
#include <ctime>
#define NORMALSIZE 255
/**
   * @brief 延时
   */
static void _delay(int usec)
{
  clock_t now = clock();
  while(clock()-now < usec);
}

//从机状态
enum SlaveState{
  UnKnown       = -1,
  ShutDown      = 10,
  Start         = 11,
  Mapping       = 12,
  Nav           = 13,//idle
  EmergencyStop = 14,
  Error         = 15,
  Cancel        = 16,
  Pause         = 21,
  Executing     = 22,
  ManualControl = 23,
  Aborted       = 24,
  Completed     = 25,
  Planning      = 26,
  Driving       = 27
};
//小车状态
typedef struct runtimeinfo{
    int         state;		//状态
    float       pos_x;		//x轴坐标位置
    float       pos_y;		//y轴的坐标位置
    float       theta;		//z轴的旋转角度
    float       battery;	//电量
    float       vel_x;		//线速度
    float       vel_th;		//角速度
    std::string map_id;		//当前正在使用的地图id
}RuntimeInfo;
//小车信息
typedef struct robot {
  int           id;					//小车id
  std::string   hostname;		//小车的主机名
  std::string   addr;				//小车地址
  std::string   port;				//小车端口号
  double        time_delay;	//小车的往返时延
}Robot;
//小车信息
typedef struct robotsinfo {
  int         robot_id;			//小车id
  double      timestamp;		//时间戳
  RuntimeInfo info;					//小车信息
}RobotsInfo;

static const std::string __INFO       = "INFO";
static const std::string __WARN       = "WARN";
static const std::string __ERROR      = "ERROR";
static const std::string __FATAL      = "FATAL";

#endif // MYSQL_HEAD_H
