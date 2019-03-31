/*
 *  path_follower.cpp
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include <nav_msgs/Path.h>
#include "yikun_navigation/local_panner/path_follower.h"

namespace yikun_navigation {
  PathFollower::PathFollower(): tf_(NULL),
    robot_base_frame_("base_footprint"),
    global_frame_("odom"),
    odom_frame_("odom"),
    started_(false),reached_(false),
    initialized_(false)
  {}

  void PathFollower::initialize(tf::TransformListener* tf){
    tf_ = tf;
    current_waypoint_ = 0;
    goal_reached_time_ = ros::Time::now();
    ros::NodeHandle node_private("~");
    
    //规划器的全局坐标系
    node_private.param("global_frame_id",global_frame_,std::string("odom"));
    //里程计坐标系
    node_private.param("odom_frame_id",odom_frame_,std::string("odom"));
    //小车坐标系
    node_private.param("base_frame_id",robot_base_frame_,std::string("base_footprint"));
    
    //速度比例
    node_private.param("k_trans", K_trans_, 1.0);
    node_private.param("k_rot", K_rot_, 1.0);

    //within this distance to the goal, finally rotate to the goal heading (also, we've reached our goal only if we're within this dist)
    node_private.param("tolerance_trans", tolerance_trans_, 0.2);

    //we've reached our goal only if we're within this angular distance
    node_private.param("tolerance_rot", tolerance_rot_, M_PI*2);

    //we've reached our goal only if we're within range for this long and stopped
    node_private.param("tolerance_timeout", tolerance_timeout_, 0.5);

    //set this to true if you're using a holonomic robot
    node_private.param("holonomic", holonomic_, false);

    //number of samples (scaling factors of our current desired twist) to check the validity of
    node_private.param("samples", samples_, 10);

    //go no faster than this
    node_private.param("max_vel_lin", max_vel_lin_, 0.35);
    node_private.param("max_vel_th", max_vel_th_, 0.8);

    //minimum velocities to keep from getting stuck
    node_private.param("min_vel_lin", min_vel_lin_, 0.05);
    node_private.param("min_vel_th", min_vel_th_, 0.0);

    //if we're rotating in place, go at least this fast to avoid getting stuck
    node_private.param("min_in_place_vel_th", min_in_place_vel_th_, 0.0);

    //when we're near the end and would be trying to go no faster than this translationally, just rotate in place instead
    node_private.param("in_place_trans_vel", in_place_trans_vel_, 0.0);

    //we're "stopped" if we're going slower than these velocities
    node_private.param("trans_stopped_velocity", trans_stopped_velocity_, 1e-4);
    node_private.param("rot_stopped_velocity", rot_stopped_velocity_, 1e-4);

    //if this is true, we don't care whether we go backwards or forwards
    node_private.param("allow_backwards", allow_backwards_, false);

    //if this is true, turn in place to face the new goal instead of arcing toward it
    node_private.param("turn_in_place_first", turn_in_place_first_, false);

    //if turn_in_place_first is true, turn in place if our heading is more than this far from facing the goal location
    node_private.param("max_heading_diff_before_moving", max_heading_diff_before_moving_, 0.17);
    
    //路径发布接口定义
    global_plan_pub_ = node_private.advertise<nav_msgs::Path>("global_plan", 1);

    ros::NodeHandle node;
    //里程计监听
    odom_sub_ = node.subscribe<nav_msgs::Odometry>("/odom", 1, boost::bind(&PathFollower::odomCallback, this, _1));
    //路径监听
    path_service_ = node.advertiseService("/set_path",&PathFollower::pathCallback,this);
    //速度发布接口定义
    vel_pub_ = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    initialized_ = true;
    ROS_INFO_STREAM("Initialized");
  }

  void PathFollower::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    //we assume that the odometry is published in the frame of the base
    boost::mutex::scoped_lock lock(odom_lock_);
    ROS_INFO_ONCE("odom");
    base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    ROS_DEBUG("In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
        base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);
  }

  bool PathFollower::pathCallback(yikun_cluster_msgs::SetPath::Request& request,
                    yikun_cluster_msgs::SetPath::Response& response)
  {
    boost::mutex::scoped_lock lock(path_lock_);
    if(!this->isInitialized()) {
      ROS_WARN_STREAM("initialize");
      response.result = 0;
      return true;
    }
    response.result = this->setPlan(request.path.poses);
    return true;
  }
  
  //计算z轴转速
  double PathFollower::headingDiff(double x, double y, double pt_x, double pt_y, double heading)
  {
    double v1_x = x - pt_x;
    double v1_y = y - pt_y;
    double v2_x = cos(heading);
    double v2_y = sin(heading);

    double perp_dot = v1_x * v2_y - v1_y * v2_x;
    double dot = v1_x * v2_x + v1_y * v2_y;

    //get the signed angle
    double vector_angle = atan2(perp_dot, dot);

    return -1.0 * vector_angle;
  }

  bool PathFollower::stopped(){
    //copy over the odometry information
    nav_msgs::Odometry base_odom;
    {
      boost::mutex::scoped_lock lock(odom_lock_);
      base_odom = base_odom_;
    }

    return fabs(base_odom.twist.twist.angular.z) <= rot_stopped_velocity_
      && fabs(base_odom.twist.twist.linear.x) <= trans_stopped_velocity_
      && fabs(base_odom.twist.twist.linear.y) <= trans_stopped_velocity_;
  }

  void PathFollower::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path,
                               const ros::Publisher &pub) {
    // given an empty path we won't do anything
    if (path.empty())
      return;

    // create a path message
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());
    gui_path.header.frame_id = path[0].header.frame_id;
    gui_path.header.stamp = path[0].header.stamp;

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
      gui_path.poses[i] = path[i];
    }
    pub.publish(gui_path);//发布
  }

  bool PathFollower::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    //get the current pose of the robot in the fixed frame
    tf::Stamped<tf::Pose> robot_pose;
    if(!this->getRobotPose(robot_pose)){//获取小车位置
      ROS_ERROR("Can't get robot pose");
      geometry_msgs::Twist empty_twist;
      cmd_vel = empty_twist;
      return false;
    }

    //we want to compute a velocity command based on our current waypoint
    tf::Stamped<tf::Pose> target_pose;
    tf::poseStampedMsgToTF(global_plan_[current_waypoint_], target_pose);

    //get the difference between the two poses
    geometry_msgs::Twist diff = diff2D(target_pose, robot_pose);
    ROS_DEBUG("PathFollower: diff %f %f ==> %f", diff.linear.x, diff.linear.y, diff.angular.z);
    //速度限制
    geometry_msgs::Twist limit_vel = limitTwist(diff);

    //if it is legal... we'll pass it on
    cmd_vel = limit_vel;

    bool in_goal_position = false;
    while(fabs(diff.linear.x) <= tolerance_trans_ &&
          fabs(diff.linear.y) <= tolerance_trans_ &&
    fabs(diff.angular.z) <= tolerance_rot_)
    {
      if(current_waypoint_ < global_plan_.size() - 1)
      {
        current_waypoint_++;
        tf::poseStampedMsgToTF(global_plan_[current_waypoint_], target_pose);
        diff = diff2D(target_pose, robot_pose);
      }
      else
      {
        ROS_INFO("Reached goal: %d", current_waypoint_);
        in_goal_position = true;
        break;
      }
    }

    //if we're not in the goal position, we need to update time
    if(!in_goal_position)
      goal_reached_time_ = ros::Time::now();

    //check if we've reached our goal for long enough to succeed
    if(goal_reached_time_ + ros::Duration(tolerance_timeout_) < ros::Time::now()){
      ROS_INFO_STREAM("goal position");
      geometry_msgs::Twist empty_twist;
      cmd_vel = empty_twist;
      reached_ = true;
    }

    return true;
  }

  bool PathFollower::available()
  {
    return started_;
  }
  bool PathFollower::isInitialized()
  {
    return initialized_;
  }

  bool PathFollower::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan){
    current_waypoint_ = 0;
    goal_reached_time_ = ros::Time::now();
    if(!transformGlobalPlan(*tf_, global_plan, odom_frame_, global_plan_)){
      ROS_ERROR("Could not transform the global plan to the frame of the controller");
      return false;
    }

    ROS_INFO("global plan size: %lu", global_plan_.size());
    publishPlan(global_plan_, global_plan_pub_);
    started_ = true;
    while(ros::ok())
    {
      geometry_msgs::Twist twist;
      //判断规划是否完成
      if(reached_ || this->isGoalReached()){
        ROS_INFO_STREAM("done");
        break;
      }
      //速度计算
      if(this->computeVelocityCommands(twist)) {
        vel_pub_.publish(twist);
      } else {
        this->publishZeroVelocity();
        ROS_WARN_STREAM("fail to get velocity");
      }
      ros::Duration(0.05).sleep();//规划器执行频率（1/T）
    }
    reached_ = false;
    return true;
  }
  void PathFollower::publishZeroVelocity()
  {
      geometry_msgs::Twist cmd_vel;
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0.0;
      vel_pub_.publish(cmd_vel);
  }

  bool PathFollower::isGoalReached(){
    if(goal_reached_time_ + ros::Duration(tolerance_timeout_) < ros::Time::now() && stopped()){
      started_ = false;
      reached_ = true;
      return true;
    }
    return false;
  }

  geometry_msgs::Twist PathFollower::diff2D(const tf::Pose& pose1, const tf::Pose& pose2)
  {
    geometry_msgs::Twist res;
    tf::Pose diff = pose2.inverse() * pose1;
    res.linear.x = diff.getOrigin().x();
    res.linear.y = diff.getOrigin().y();
    res.angular.z = tf::getYaw(diff.getRotation());

    if(holonomic_ || (fabs(res.linear.x) <= tolerance_trans_ && fabs(res.linear.y) <= tolerance_trans_))
      return res;

    //in the case that we're not rotating to our goal position and we have a non-holonomic robot
    //we'll need to command a rotational velocity that will help us reach our desired heading

    //we want to compute a goal based on the heading difference between our pose and the target
    double yaw_diff = headingDiff(pose1.getOrigin().x(), pose1.getOrigin().y(),
        pose2.getOrigin().x(), pose2.getOrigin().y(), tf::getYaw(pose2.getRotation()));

    //we'll also check if we can move more effectively backwards
    if (allow_backwards_)
    {
      double neg_yaw_diff = headingDiff(pose1.getOrigin().x(), pose1.getOrigin().y(),
          pose2.getOrigin().x(), pose2.getOrigin().y(), M_PI + tf::getYaw(pose2.getRotation()));

      //check if its faster to just back up
      if(fabs(neg_yaw_diff) < fabs(yaw_diff)){
        ROS_DEBUG("Negative is better: %.2f", neg_yaw_diff);
        yaw_diff = neg_yaw_diff;
      }
    }

    //compute the desired quaterion
    tf::Quaternion rot_diff = tf::createQuaternionFromYaw(yaw_diff);
    tf::Quaternion rot = pose2.getRotation() * rot_diff;
    tf::Pose new_pose = pose1;
    new_pose.setRotation(rot);

    diff = pose2.inverse() * new_pose;
    res.linear.x = diff.getOrigin().x();
    res.linear.y = diff.getOrigin().y();
    res.angular.z = tf::getYaw(diff.getRotation());
    return res;
  }


  geometry_msgs::Twist PathFollower::limitTwist(const geometry_msgs::Twist& twist)
  {
    geometry_msgs::Twist res = twist;
    res.linear.x *= K_trans_;
    if(!holonomic_)
      res.linear.y = 0.0;
    else
      res.linear.y *= K_trans_;
    res.angular.z *= K_rot_;

    //if turn_in_place_first is true, see if we need to rotate in place to face our goal first
    if (turn_in_place_first_ && fabs(twist.angular.z) > max_heading_diff_before_moving_)
    {
      res.linear.x = 0;
      res.linear.y = 0;
      if (fabs(res.angular.z) > max_vel_th_) res.angular.z = max_vel_th_ * sign(res.angular.z);
      if (fabs(res.angular.z) < min_in_place_vel_th_) res.angular.z = min_in_place_vel_th_ * sign(res.angular.z);
      return res;
    }

    //make sure to bound things by our velocity limits
    double lin_overshoot = sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y) / max_vel_lin_;
    double lin_undershoot = min_vel_lin_ / sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y);
    if (lin_overshoot > 1.0)
    {
      res.linear.x /= lin_overshoot;
      res.linear.y /= lin_overshoot;
    }

    //we only want to enforce a minimum velocity if we're not rotating in place
    if(lin_undershoot > 1.0)
    {
      res.linear.x *= lin_undershoot;
      res.linear.y *= lin_undershoot;
    }

    if (fabs(res.angular.z) > max_vel_th_) res.angular.z = max_vel_th_ * sign(res.angular.z);
    if (fabs(res.angular.z) < min_vel_th_) res.angular.z = min_vel_th_ * sign(res.angular.z);
    if (std::isnan(res.linear.x))
        res.linear.x = 0.0;
    if (std::isnan(res.linear.y))
        res.linear.y = 0.0;

    //we want to check for whether or not we're desired to rotate in place
    if(sqrt(twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y) < in_place_trans_vel_){
      if (fabs(res.angular.z) < min_in_place_vel_th_) res.angular.z = min_in_place_vel_th_ * sign(res.angular.z);
      res.linear.x = 0.0;
      res.linear.y = 0.0;
    }

    ROS_DEBUG("Angular command %f", res.angular.z);
    return res;
  }

  bool PathFollower::transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
      const std::string& global_frame,
      std::vector<geometry_msgs::PoseStamped>& transformed_plan){
    const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

    transformed_plan.clear();

    try{
      if (global_plan.empty())
      {
        ROS_ERROR("Recieved plan with zero length");
        return false;
      }

      tf::StampedTransform transform;
      bool succeed = tf.waitForTransform(global_frame, ros::Time::now(),
                                         plan_pose.header.frame_id, plan_pose.header.stamp,
                                         plan_pose.header.frame_id, ros::Duration(1.0));
      if(!succeed)
      {
          ROS_ERROR("Transform error!");
          return false;
      }
      tf.lookupTransform(global_frame, ros::Time(),
          plan_pose.header.frame_id, plan_pose.header.stamp,
          plan_pose.header.frame_id, transform);

      tf::Stamped<tf::Pose> tf_pose;
      geometry_msgs::PoseStamped newer_pose;
      //now we'll transform until points are outside of our distance threshold
      for(unsigned int i = 0; i < global_plan.size(); ++i){
        const geometry_msgs::PoseStamped& pose = global_plan[i];
        poseStampedMsgToTF(pose, tf_pose);
        tf_pose.setData(transform * tf_pose);
        tf_pose.stamp_ = transform.stamp_;
        tf_pose.frame_id_ = global_frame;
        poseStampedTFToMsg(tf_pose, newer_pose);

        transformed_plan.push_back(newer_pose);
      }
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      if (!global_plan.empty())
        ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

      return false;
    }

    return true;
  }
  bool PathFollower::getRobotPose(tf::Stamped<tf::Pose>& global_pose) const {

    global_pose.setIdentity();
    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = robot_base_frame_;
    robot_pose.stamp_ = ros::Time(0);

    //get the global pose of the robot
    try{
      tf_->transformPose(odom_frame_, robot_pose, global_pose);
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    return true;
  }
}
