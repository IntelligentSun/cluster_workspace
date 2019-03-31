/*
 *  service.h
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#ifndef SERVICE_H
#define SERVICE_H

#include "yikun_common/service/header.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include "yikun_cluster_msgs/GetState.h"
#include "yikun_cluster_msgs/SetPose.h"
#include "yikun_cluster_msgs/SetState.h"
#include "yikun_cluster_msgs/DockPose.h"
#include "yikun_cluster_msgs/SetPath.h"
#include "yikun_common/mysql/db_helper.h"

namespace yikun_common {
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> _MoveBaseClient;
class Service
{
public:
  Service(ros::NodeHandle &nh);
  ~Service();
  /**
   * @brief 开始线程 
   */
  void start();
  /**
   * @brief 结束线程 
   */
  void stop();
  /**
   * @brief 手动校准 
   * @param req	请求
   * @param res 响应
   */
  bool correctLocation(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
  /**
   * @brief 手动设置小车位置
   * @param req	请求
   * @param res 响应
   */
  bool setPose(yikun_cluster_msgs::SetPoseRequest &req, yikun_cluster_msgs::SetPoseResponse &res);
  /**
   * @brief 移动到目标点 
   */
  void simpleGoal(yikun_cluster_msgs::DockPoseConstPtr msg);
	
  static void *moveto(void *arg);
  /**
   * @brief 移动到目标位置 
   */
  bool sendGoal(const _PoseStamped&);
  /**
   * @brief 解析路径
   * @param p 接收到的路径
   * @param path 标准格式的路径
   */
  bool getPath(const char* p,std::vector<geometry_msgs::PoseStamped> &path);
  /**
   * @brief 设置路径
   * @param p 路径
   */
  bool setPath(const char *p);
private:
  //communication interface
  /**
   * @brief 目标点监听线程 
   */
  void goalServiceServer();//send goal(TCP)
  /**
   * @brief 路径监听线程 
   */
  void setPathServer();//
  void PathSubcriber();
  /**
   * @brief 速度监听线程
   */
  void velocitySubcriber();//wait velocity(UDP)
  /**
   * @brief UWB定位监听线程 
   */
  void uwbPoseSubcriber();//wait uwb pose(UDP)
  /**
   * @brief Move Base 监听线程
   */
  void movebaseSuperviser();
  /**
   * @brief 发布路径
   * @param path 路径
   * @param pub 发布器接口 
   */
  void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path,
                            const ros::Publisher& pub);

  ros::NodeHandle nh_;
  ros::NodeHandle *private_nh_;
  //线程
  boost::thread *vel_sub_;
  boost::thread *uwb_sub_;
  boost::thread *service_call_;
  boost::thread *path_gen_service_;
  boost::thread *move_base_superviser_;
  
  _MoveBaseClient *move_base_;
  ros::Publisher set_pose_;
  ros::Publisher vel_pub_;
  ros::Publisher uwb_pose_pub_;
  ros::Publisher e_stop_;
  ros::Publisher simple_goal_pub_;
  ros::Publisher plan_pub_;
  ros::Subscriber simple_goal_sub_;
  ros::ServiceClient correct_client_;
  ros::ServiceServer set_pose_server_,correct_server_;
  ros::ServiceClient set_plan_;
  bool is_navigation_,started_;
  bool interrupt_;//中断
  std::string addr_;
  boost::recursive_mutex recursive_mutex_;
  boost::mutex mutex_;
  struct timeval timeout_;
  std::string global_frame_id_,base_frame_id_,sensor_frame_id_;
};

}//namespace

#endif // SERVICE_H
