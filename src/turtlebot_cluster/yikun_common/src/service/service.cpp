/*
 *  service.cpp
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include "yikun_common/service/service.h"
#include <tf/tf.h>
#include <memory>
#include <nav_msgs/Odometry.h>

namespace yikun_common {

Service::Service(ros::NodeHandle &nh)
  : nh_(nh),started_(false),global_frame_id_("map"),base_frame_id_("base_footprint"),
//    sensor_frame_id_("uwb_link"),
    vel_sub_(NULL),service_call_(NULL),move_base_superviser_(NULL)
{
  private_nh_ = new ros::NodeHandle("~");
  //服务地址
  private_nh_->param("addr",addr_,std::string("0.0.0.0"));
  int timesec;
  private_nh_->param("timesec",timesec,60);
  //世界坐标
  private_nh_->param("global_frame_id",global_frame_id_,std::string("map"));
  //小车坐标  
  private_nh_->param("base_frame_id",base_frame_id_,std::string("base_footprint"));
  //uwb坐标
//  private_nh_->param("sensor_frame_id",sensor_frame_id_,std::string("uwb_link"));
  //movebase action
  move_base_ = new _MoveBaseClient("move_base",true);
  
  //service client
  //手动位置校准
  correct_client_ = private_nh_->serviceClient<std_srvs::Empty>("/request_adjust");
  //路径设置请求接口定义
  set_plan_ = private_nh_->serviceClient<yikun_cluster_msgs::SetPath>("/set_path");
  //publisher
  //amcl 位置初始化接口
  set_pose_ = private_nh_->advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1);
  //小车速度控制接口  
  vel_pub_ = private_nh_->advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",100);
//  uwb_pose_pub_ = private_nh_->advertise<geometry_msgs::PoseWithCovarianceStamped>("uwb_pose",1000);
  //UWB数据发布接口  
  uwb_pose_pub_ = private_nh_->advertise<nav_msgs::Odometry>("/odometry/uwb",10);
  //  e_stop_ = private_nh_->advertise<std_msgs::Bool>("/e_stop",1);
  simple_goal_pub_ = private_nh_->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
  plan_pub_ = private_nh_->advertise<nav_msgs::Path>("global_path",1);
  //subcriber
  //前往接口（WEB）
  simple_goal_sub_ = private_nh_->subscribe<yikun_cluster_msgs::DockPose>("/yikun/simple_goal",1,boost::bind(&Service::simpleGoal,this,_1));
  //service server
  //设置初始位置（WEB）
  set_pose_server_ = private_nh_->advertiseService("/yikun/set_pose",&Service::setPose,this);
  //手动位置校准（WEB）
  correct_server_ = private_nh_->advertiseService("/yikun/correct_location",&Service::correctLocation,this);
  //timeout
  timeout_.tv_sec = timesec;
  timeout_.tv_usec = 0;
}

Service::~Service()
{
  //线程销毁
  vel_sub_->interrupt();
  uwb_sub_->interrupt();
  service_call_->interrupt();
  path_gen_service_->interrupt();
  move_base_superviser_->interrupt();
	vel_sub_->join();
  uwb_sub_->join();
  service_call_->join();
  path_gen_service_->join();
  move_base_superviser_->join();
  delete service_call_,path_gen_service_;
  delete move_base_superviser_,
  delete vel_sub_,uwb_sub_;
  vel_sub_ = uwb_sub_ = service_call_ = path_gen_service_ = move_base_superviser_ = NULL;
}

void Service::start()
{
  if(started_) {
    std::perror("is running");
    return;
  }
  //线程初始化
  move_base_superviser_ = new boost::thread(boost::bind(&Service::movebaseSuperviser,this));
  vel_sub_ = new boost::thread(boost::bind(&Service::velocitySubcriber,this));
  uwb_sub_ = new boost::thread(boost::bind(&Service::uwbPoseSubcriber,this));
  service_call_ = new boost::thread(boost::bind(&Service::goalServiceServer,this));
//  path_gen_service_ = new boost::thread(boost::bind(&Service::setPathServer,this));
  path_gen_service_ = new boost::thread(boost::bind(&Service::PathSubcriber,this));

  started_ = true;
}

void Service::stop()
{
  if(started_) {
    return;
  }
  //线程销毁
  vel_sub_->interrupt();
  uwb_sub_->interrupt();
  service_call_->interrupt();
  path_gen_service_->interrupt();
  move_base_superviser_->interrupt();
  vel_sub_->join();
  uwb_sub_->join();
  service_call_->join();
  path_gen_service_->join();
  move_base_superviser_->join();
  delete service_call_,path_gen_service_;
  delete move_base_superviser_,
  delete vel_sub_,uwb_sub_;
  vel_sub_ = uwb_sub_ = service_call_ = path_gen_service_ = move_base_superviser_ = NULL;
}

void Service::goalServiceServer()
{
  boost::this_thread::interruption_point();
  constexpr int listen_port = TCP_PORT;
  int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (sock == -1)
  {
    std::perror("socket");
    return;
  }
  //timeout
  //设置请求超时
  if(setsockopt(sock,SOL_SOCKET,SO_SNDTIMEO,(const char*)&timeout_,sizeof(timeout_)) == -1)
    return;
  if(setsockopt(sock,SOL_SOCKET,SO_RCVTIMEO,(const char*)&timeout_,sizeof(timeout_)) == -1)
    return;

  sockaddr_in client;
  socklen_t len = sizeof(client);
  sockaddr_in addr;

  std::memset(&addr, 0, sizeof(sockaddr_in));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(listen_port);
  addr.sin_addr.s_addr = inet_addr(addr_.data());
  int ret = bind(sock, (sockaddr *)&addr, sizeof(addr));
  if (ret == -1)
  {
    std::perror("bind");
    goto error;
  }
//  std::cout<<__func__<<" bind addr "<<addr_<<std::endl;
  ret = listen(sock, 1);
  if (ret == -1)
  {
    std::perror("listen");
    goto error;
  }

  while (nh_.ok())
  {
    // get connect
//    std::cout << "Waiting tcp access..." << std::endl;
//    boost::unique_lock<boost::recursive_mutex> lock(recursive_mutex_);
    //等待连接请求
    int *client_sock = new int();
    *client_sock = accept(sock, reinterpret_cast<sockaddr *>(&client), &len);
    if (*client_sock == -1)
    {
      std::perror("accept goal");
      continue;
    }
    //just one thread
    _PoseStamped pose;
    std::memset(&pose,0,sizeof(_PoseStamped));
    ssize_t n = recv(*client_sock, &pose, sizeof(_PoseStamped), 0); //获取位置点数据
    if(n < 0 && errno == EAGAIN){
      std::perror("recv");
      continue;
    }
//    ROS_INFO("%s,%.4f,%.4f,%.4f",pose.frame_id,pose.orientation.w,pose.position.x,pose.stamp);
    //request
    if(this->sendGoal(pose)) {
      pose.ack = ACK_SUCCEED;
    } else {
      pose.ack = ACK_FAILED;
    }
    pose.stamp = ros::Time::now().toSec();
    //response
    n = send(*client_sock, &pose, sizeof(_PoseStamped), 0); //发送响应数据
    if(n < 0 && errno == EAGAIN){
      std::perror("write");
      continue;
    }
    //multithread
    //.....
  }
  //end
  boost::this_thread::interruption_enabled();
  return;
error:
  close(sock);
  boost::this_thread::interruption_enabled();
  return;

}

void Service::setPathServer()
{
  boost::this_thread::interruption_point();
  constexpr int listen_port = PATH_PORT;
  int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (sock == -1)
  {
    std::perror("socket");
    return;
  }
  //timeout
  struct timeval timeout;
  timeout.tv_sec = 600;
  timeout.tv_usec = 0;
  if(setsockopt(sock,SOL_SOCKET,SO_SNDTIMEO,(const char*)&timeout,sizeof(timeout_)) == -1)
    return;
  if(setsockopt(sock,SOL_SOCKET,SO_RCVTIMEO,(const char*)&timeout,sizeof(timeout_)) == -1)
    return;

  sockaddr_in client;
  socklen_t len = sizeof(client);
  sockaddr_in addr;

  std::memset(&addr, 0, sizeof(sockaddr_in));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(listen_port);
  addr.sin_addr.s_addr = inet_addr(addr_.data());
  int ret = bind(sock, (sockaddr *)&addr, sizeof(addr));
  if (ret == -1)
  {
    std::perror("bind");
    goto error;
  }
//  std::cout<<__func__<<" bind addr "<<addr_<<std::endl;
  ret = listen(sock, 1);
  if (ret == -1)
  {
    std::perror("listen");
    goto error;
  }

  while (nh_.ok())
  {
    _Paths paths;
    // get connect
    int *client_sock = new int();
    *client_sock = accept(sock, reinterpret_cast<sockaddr *>(&client), &len);
    if (*client_sock == -1)
    {
      std::perror("accept path");
      continue;
    }
    //just one thread
    std::memset(&paths,0,sizeof(_Paths));
    ssize_t n = recv(*client_sock, &paths, sizeof(_Paths), 0);
    if(n < 0 && errno == EAGAIN){
      std::perror("recv");
      continue;
    }
    //request
    if(this->setPath(paths.paths)) {
      std::memset(&paths,0,sizeof(_Paths));
      paths.ack = ACK_SUCCEED;
    } else {
      std::memset(&paths,0,sizeof(_Paths));
      paths.ack = ACK_FAILED;
    }
    //response
    n = send(*client_sock, &paths, sizeof(_Paths), 0);
    if(n < 0 && errno == EAGAIN){
      std::perror("write");
      continue;
    }
    std::memset(&paths,0,sizeof(_Paths));
    //multithread
    //.....
  }
  //end
  boost::this_thread::interruption_enabled();
  return;
error:
  close(sock);
  boost::this_thread::interruption_enabled();
  return;

}

void Service::velocitySubcriber()
{
  boost::this_thread::interruption_point();
  constexpr int listen_port = UDP_PORT;
  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock == -1)
  {
    std::perror("socket");
    return;
  }
  //timeout
//  if(setsockopt(sock,SOL_SOCKET,SO_SNDTIMEO,(const char*)&timeout_,sizeof(timeout_)) == -1)
//    return;
//  if(setsockopt(sock,SOL_SOCKET,SO_RCVTIMEO,(const char*)&timeout_,sizeof(timeout_)) == -1)
//    return;

  sockaddr_in client;
  socklen_t len = sizeof(client);
  sockaddr_in addr;

  std::memset(&addr, 0, sizeof(sockaddr_in));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(listen_port);
  addr.sin_addr.s_addr = inet_addr(addr_.data());
  int ret = bind(sock, (sockaddr *)&addr, sizeof(addr));
  if (ret == -1)
  {
    std::perror("bind");
//    goto error;
    return;
  }
  while (nh_.ok())
  {
    // get connect
//    std::cout << "Waiting udp access..." << std::endl;
//    boost::unique_lock<boost::recursive_mutex> lock(recursive_mutex_);
    int *client_sock = new int();
    _twist twist;
    std::memset(&twist,0,sizeof(_twist));
    *client_sock = recvfrom(sock, &twist, sizeof(_twist), 0,reinterpret_cast<sockaddr *>(&client), &len);
    if (*client_sock == -1 && errno == EAGAIN)
    {
      std::perror("recv");
      continue;
    }
    //response
    geometry_msgs::Twist vel;
    vel.linear.x = twist.linear_x;
    vel.angular.z = twist.angular_z;
//    std::cout<<twist.linear_x<<"\t"<<twist.angular_z<<std::endl;
    vel_pub_.publish(vel);
  }
  //end
  boost::this_thread::interruption_enabled();
  return;
//error:
//  close(sock);
//  boost::this_thread::interruption_enabled();
//  return;
}

void Service::PathSubcriber()
{
  boost::this_thread::interruption_point();
  constexpr int listen_port = PATH_PORT;
  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock == -1)
  {
    std::perror("socket");
    return;
  }
  sockaddr_in client;
  socklen_t len = sizeof(client);
  sockaddr_in addr;

  std::memset(&addr, 0, sizeof(sockaddr_in));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(listen_port);
  addr.sin_addr.s_addr = inet_addr(addr_.data());
  int ret = bind(sock, (sockaddr *)&addr, sizeof(addr));
  if (ret == -1)
  {
    std::perror("bind");
//    goto error;
    return;
  }
  while (nh_.ok())
  {
    // get connect
    int *client_sock = new int();
    _Paths path;
    std::memset(&path,0,sizeof(_Paths));
    *client_sock = recvfrom(sock, &path, sizeof(_Paths), 0,reinterpret_cast<sockaddr *>(&client), &len);
    if (*client_sock == -1 && errno == EAGAIN)
    {
      std::perror("recv");
      continue;
    }
    //request
    if(this->setPath(path.paths)) {
      std::memset(&path,0,sizeof(_Paths));
      path.ack = ACK_SUCCEED;
    } else {
      std::memset(&path,0,sizeof(_Paths));
      path.ack = ACK_FAILED;
    }
    //no response
  }
  //end
  boost::this_thread::interruption_enabled();
  return;
}

void Service::uwbPoseSubcriber()
{
  boost::this_thread::interruption_point();
  constexpr int listen_port = UWB_PORT;
  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock == -1)
  {
    std::perror("socket");
    return;
  }
  //timeout
//  if(setsockopt(sock,SOL_SOCKET,SO_SNDTIMEO,(const char*)&timeout_,sizeof(timeout_)) == -1)
//    return;
//  if(setsockopt(sock,SOL_SOCKET,SO_RCVTIMEO,(const char*)&timeout_,sizeof(timeout_)) == -1)
//    return;

  sockaddr_in client;
  socklen_t len = sizeof(client);
  sockaddr_in addr;

  std::memset(&addr, 0, sizeof(sockaddr_in));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(listen_port);
  addr.sin_addr.s_addr = inet_addr(addr_.data());
  int ret = bind(sock, (sockaddr *)&addr, sizeof(addr));
  if (ret == -1)
  {
    std::perror("bind");
//    goto error;
    return;
  }
  while (nh_.ok())
  {
    // get connect
//    std::cout << "Waiting udp access..." << std::endl;
//    boost::unique_lock<boost::recursive_mutex> lock(recursive_mutex_);
    int *client_sock = new int();
    _PoseWithCovarianceStamped pose;
    std::memset(&pose,0,sizeof(_PoseWithCovarianceStamped));
    *client_sock = recvfrom(sock, &pose, sizeof(_PoseWithCovarianceStamped), 0,reinterpret_cast<sockaddr *>(&client), &len);
    if (*client_sock == -1 && errno == EAGAIN)
    {
      std::perror("recv");
      continue;
    }
    //response
//    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    nav_msgs::Odometry uwb_odom;
    ros::Time time;
    time.fromSec(pose.stamp);
    uwb_odom.header.stamp = time;
    uwb_odom.header.frame_id = pose.frame_id;
    uwb_odom.child_frame_id = base_frame_id_;
//    uwb_odom.child_frame_id = sensor_frame_id_;
    uwb_odom.pose.pose.position.x = pose.position.x;
    uwb_odom.pose.pose.position.y = pose.position.y;
    uwb_odom.pose.pose.position.z = pose.position.z;
    uwb_odom.pose.pose.orientation.w = pose.orientation.w;
    uwb_odom.pose.pose.orientation.x = pose.orientation.x;
    uwb_odom.pose.pose.orientation.y = pose.orientation.y;
    uwb_odom.pose.pose.orientation.z = pose.orientation.z;
    for(int i=0;i<36;i++) {
      uwb_odom.pose.covariance[i] = pose.covariance[i];
    }
    uwb_pose_pub_.publish(uwb_odom);
  }
  //end
  boost::this_thread::interruption_enabled();
  return;
//error:
//  close(sock);
//  boost::this_thread::interruption_enabled();
//  return;
}

bool Service::getPath(const char *p,std::vector<geometry_msgs::PoseStamped> &path)
{
  path.clear();

  //路径解析，将路径转化为标准路径格式
  std::string pathstr = p;
  std::vector<std::string> points;
  if(pathstr.find(';') == std::string::npos)
    return false;
  boost::split(points,pathstr,boost::is_any_of(";"));
  for(std::string point : points)
  {
    std::vector<std::string> index;
    geometry_msgs::PoseStamped pose;
    if(point.find(',') == std::string::npos)
      return true;
    boost::split(index,point,boost::is_any_of(","));
    pose.header.frame_id = global_frame_id_;
    pose.header.stamp = ros::Time::now();
    //（x，y）根据实际路径文件赋值
    pose.pose.position.x = atof(index[2].c_str())*0.01;	//x
    pose.pose.position.y = atof(index[1].c_str())*0.01;	//y
    //姿态初始化，UWB数据中无姿态
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    path.push_back(pose);
  }
  return true;
}

bool Service::setPath(const char *p)
{
  std::vector<geometry_msgs::PoseStamped> path;
  if(getPath(p,path)) {
    this->publishPlan(path,plan_pub_);
  }
}

bool Service::sendGoal(const _PoseStamped &pos)
{
  if(!is_navigation_) {
    sleep(5);
    return false;
  }
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = pos.frame_id;
  goal.target_pose.header.stamp.sec = pos.stamp;
  goal.target_pose.pose.position.x = pos.position.x;
  goal.target_pose.pose.position.y = pos.position.y;
  goal.target_pose.pose.position.z = pos.position.z;
  goal.target_pose.pose.orientation.w = pos.orientation.w;
  goal.target_pose.pose.orientation.x = pos.orientation.x;
  goal.target_pose.pose.orientation.y = pos.orientation.y;
  goal.target_pose.pose.orientation.z = pos.orientation.z;
  move_base_->sendGoal(goal);
  move_base_->waitForResult();
  if(move_base_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    return true;
  } else {
    return false;
  }
}

void Service::movebaseSuperviser()
{
  boost::this_thread::interruption_point();
  while(nh_.ok())
  {
    is_navigation_ = move_base_->waitForServer(ros::Duration(1.0));	//检查Move Base连接
  }
  //end
  boost::this_thread::interruption_enabled();
}

bool Service::correctLocation(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
  //if robot state is navigation
  if(is_navigation_)
  {
    std_srvs::Empty call;
    for(int i = 0;i<10;i++)//attempting to request 10 times
    {
      correct_client_.call(call);//request amcl
      ros::Duration(0.1).sleep();
    }
    return true;
  }
  else {
    ROS_ERROR_STREAM("MoveBase is shutdown");
    return false;
  }
}

bool Service::setPose(yikun_cluster_msgs::SetPoseRequest &req, yikun_cluster_msgs::SetPoseResponse &res)
{
  if(!is_navigation_) {
    res.result = 0;
    ROS_ERROR_STREAM("MoveBase is shutdown");
  }
  geometry_msgs::PoseWithCovarianceStamped pos;
  pos.header.frame_id = "map";
  pos.header.stamp = ros::Time::now();
  pos.pose.pose.position.x = req.x;
  pos.pose.pose.position.y = req.y;
  tf::Quaternion q = tf::createQuaternionFromYaw((double)(req.theta*M_PI/180));
  pos.pose.pose.orientation.w = q.getW();
  pos.pose.pose.orientation.x = q.getX();
  pos.pose.pose.orientation.y = q.getY();
  pos.pose.pose.orientation.z = q.getZ();
  pos.pose.covariance[6*0+0] = 0.5 * 0.5;
  pos.pose.covariance[6*1+1] = 0.5 * 0.5;
  pos.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
  set_pose_.publish(pos);
  ros::Duration(0.1).sleep();//sleep
  res.result = 1;
  return true;
}

void Service::simpleGoal(yikun_cluster_msgs::DockPoseConstPtr msg)
{
  if(!is_navigation_) {
    ROS_ERROR_STREAM("MoveBase is shutdown");
  }
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "map";
  pose.pose.position.x = msg->x;
  pose.pose.position.y = msg->y;
  tf::Quaternion q = tf::createQuaternionFromYaw((double)(msg->theta*M_PI/180));
  pose.pose.orientation.w = q.getW();
  pose.pose.orientation.x = q.getX();
  pose.pose.orientation.y = q.getY();
  pose.pose.orientation.z = q.getZ();
  simple_goal_pub_.publish(pose);
  ros::Duration(0.1).sleep();//sleep
}

void *Service::moveto(void *arg)
{
  ROS_INFO_STREAM("service call begin");
  int *client_sockp = static_cast<int *>(arg);
  int sock = *client_sockp;
  delete client_sockp;
  _PoseStamped *pos = (_PoseStamped*)malloc(sizeof(_PoseStamped));
  int len = sizeof(pos);
  char tmp[len];
  if(true) {
    ssize_t n = recv(sock, tmp, len, 0);
    if(n < 0){
      std::perror("recv");
      return nullptr;
    }
    memcpy(pos,tmp,len);

    pos->ack = ACK_SUCCEED;
    memset(tmp,0,len);
    memcpy(tmp,pos,len);
    n = write(sock, tmp, sizeof(tmp));
    if(n < 0){
      std::perror("write");
      return nullptr;
    }
  }

  if (close(sock) < 0)
  {
    std::perror("close");
    return nullptr;
  }
  ROS_INFO_STREAM("service call end");
  return nullptr;
}

void Service::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub)
{
  //given an empty path we won't do anything
  if(path.empty())
    return;

  //create a path message
  nav_msgs::Path gui_path;
  gui_path.poses.resize(path.size());
  gui_path.header.frame_id = path[0].header.frame_id;
  gui_path.header.stamp = path[0].header.stamp;

  // Extract the plan in world co-ordinates, we assume the path is all in the same frame
  for(unsigned int i=0; i < path.size(); i++){
    gui_path.poses[i] = path[i];
  }
  yikun_cluster_msgs::SetPath client;
  client.request.path = gui_path;
  if(!set_plan_.call(client))
    ROS_WARN_STREAM("set plan");
  pub.publish(gui_path);
}

}//namespace
