/*
 *  path_follower.h
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#ifndef PATH_FOLLOWER_H
#define PATH_FOLLOWER_H
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "yikun_cluster_msgs/SetPath.h"

namespace yikun_navigation {
  class PathFollower {
    public:
      typedef boost::shared_ptr<yikun_navigation::PathFollower> Ptr;
      PathFollower();
      /**
       * @brief 规划器初始化
       * @param tf 坐标转换
       */
      void initialize(tf::TransformListener* tf);
      /**
       * @brief 判断是否完成
       */
      bool isGoalReached();
      /**
       * @brief 设置全局路径
       * @param global_plan 全局路径
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);
      /**
       * @brief 计算当前小车的速度
       * @output 当前速度输出
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
      
      /**
       * @brief 判断规划器是否已经开启
       */
      bool available();
      /**
       * @brief 判断规划器是否初始化
       */
      bool isInitialized();

    private:
      inline double sign(double n){
        return n < 0.0 ? -1.0 : 1.0;
      }
      /**
       * @brief 速度计算
       * @param pose1 下一个路点位置
       * @param pose2 当前小车的位置
       */
      geometry_msgs::Twist diff2D(const tf::Pose& pose1, const tf::Pose&  pose2);
      /**
       * @brief 速度限制
       */
      geometry_msgs::Twist limitTwist(const geometry_msgs::Twist& twist);
      double headingDiff(double pt_x, double pt_y, double x, double y, double heading);
      
      /**
       * @brief 将全局路径转换为global_frame下的局部路径
       */
      bool transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
          const std::string& global_frame,
          std::vector<geometry_msgs::PoseStamped>& transformed_plan);
      /**
       * @brief 里程计监听
       */
      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
      /**
       * @brief 全局路径监听
       */
      bool pathCallback(yikun_cluster_msgs::SetPath::Request& request,
                        yikun_cluster_msgs::SetPath::Response& response);
      /**
       * @brief 将小车速度设置为0
       */
      void publishZeroVelocity();
      /**
       * @brief 停止判断
       */
      bool stopped();
      /**
       * @brief 发布当前规划器的路径
       */
      void publishPlan(const std::vector<geometry_msgs::PoseStamped> &path, const ros::Publisher &pub);
      /**
       * @brief 获取机器人当前位置
       */
      bool getRobotPose(tf::Stamped<tf::Pose>& global_pose) const;

      tf::TransformListener* tf_;
//      costmap_2d::Costmap2DROS* costmap_ros_;
      ros::Publisher vel_pub_;
      ros::Publisher global_plan_pub_;
      double K_trans_, K_rot_, tolerance_trans_, tolerance_rot_;
      double tolerance_timeout_;
      double max_vel_lin_, max_vel_th_;
      double min_vel_lin_, min_vel_th_;
      double min_in_place_vel_th_, in_place_trans_vel_;
      bool allow_backwards_;
      bool turn_in_place_first_;
      double max_heading_diff_before_moving_;
      bool holonomic_;
      boost::mutex odom_lock_,path_lock_;
      ros::Subscriber odom_sub_;
      ros::ServiceServer path_service_;
      nav_msgs::Odometry base_odom_;
      double trans_stopped_velocity_, rot_stopped_velocity_;
      ros::Time goal_reached_time_;
      unsigned int current_waypoint_;
      std::vector<geometry_msgs::PoseStamped> global_plan_;
//      base_local_planner::TrajectoryPlannerROS collision_planner_;
      int samples_;
      std::string robot_base_frame_,global_frame_,odom_frame_;
      bool started_,reached_,initialized_;
  };
}//namespace
#endif // PATH_FOLLOWER_H
