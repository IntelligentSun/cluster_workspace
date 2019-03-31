/*
 *  transmission.cpp
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "message_filters/subscriber.h"

struct TransformListenerWrapper : public tf::TransformListener
{
  inline tf2_ros::Buffer &getBuffer() {return tf2_buffer_;}
};
//global value
ros::Publisher pub_;
nav_msgs::Odometry odom_;
std::string odom_frame_id_ = "odom";
std::string base_frame_id_ = "base_footprint";
std::string sensor_frame_id_ = "uwb_link";
std::string global_frame_id_ = "map";
TransformListenerWrapper* tf_;
tf::Transform latest_tf_;
tf::TransformBroadcaster* tfb_;
ros::Duration transform_tolerance_;
//tf::Pose latest_pose_;
//接收到UWB定位
void callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  nav_msgs::Odometry pose;
  pose = *msg;
  pose.header.frame_id = global_frame_id_;
  pose.child_frame_id = base_frame_id_;
  pub_.publish(pose);
  tf::Stamped<tf::Pose> odom_to_map;
  tf::Quaternion q(0,0,0,1);
  //base_frame->global_frame的坐标转换
  tf::Transform tmp_tf(q,
                       tf::Vector3(pose.pose.pose.position.x,
                                   pose.pose.pose.position.y,
                                   0.0));
  try
  {
    tf::Stamped<tf::Pose> tmp_tf_stamped(tmp_tf.inverse(),
                                          pose.header.stamp,
                                          base_frame_id_);
    //计算odom_frame->global_frame的坐标转换
//    tf_->transformPose(odom_frame_id_, tmp_tf_stamped, odom_to_map);
    tf::StampedTransform transform;
    if(tf_->waitForTransform(odom_frame_id_, base_frame_id_, ros::Time(), ros::Duration(1.0))) {
      tf_->lookupTransform(odom_frame_id_, base_frame_id_, ros::Time(0), transform);
      odom_to_map.setData(transform * tmp_tf_stamped);
      odom_to_map.stamp_ = transform.stamp_;
      odom_to_map.frame_id_ = odom_frame_id_;
    } else {
      ROS_ERROR_STREAM("lookup transform");
      return;
    }

  }

  catch(tf::TransformException)
  {
    ROS_ERROR("Failed to subtract base to odom transform");
    return;
  }

  latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                             tf::Point(odom_to_map.getOrigin()));

  ros::Time transform_expiration = (pose.header.stamp + transform_tolerance_);
  tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                      transform_expiration,
                                      global_frame_id_, odom_frame_id_);
  //发布坐标odom_frame->global_frame的坐标转换
  tfb_->sendTransform(tmp_tf_stamped);
//  latest_pose_ = tmp_tf;
}
//
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
//  odom_ = *msg;
  static tf::TransformBroadcaster br;
//  tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,
//                   msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
  tf::Transform tmp_tf(tf::Quaternion(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,
                                      msg->pose.pose.orientation.z,msg->pose.pose.orientation.w),
                       tf::Vector3(msg->pose.pose.position.x,
                                   msg->pose.pose.position.y,
                                   0.0));
  ros::Time transform_time = msg->header.stamp;
  br.sendTransform(tf::StampedTransform(tmp_tf, transform_time, odom_frame_id_, base_frame_id_));
}

int main(int argc,char* argv[])
{
  ros::init(argc,argv,"transmission");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  //世界坐标
  private_nh.param("global_frame_id",global_frame_id_,std::string("map"));
  //里程计坐标
  private_nh.param("odom_frame_id",odom_frame_id_,std::string("odom"));
  //小车坐标
  private_nh.param("base_frame_id",base_frame_id_,std::string("base_footprint"));
  //uwb坐标
  private_nh.param("sensor_frame_id",sensor_frame_id_,std::string("uwb_link"));
  ros::MultiThreadedSpinner spinner(2);
  tfb_ = new tf::TransformBroadcaster();
  tf_ = new TransformListenerWrapper();
  transform_tolerance_.fromSec(0.5);
  //监听UWB定位系统的定位
  ros::Subscriber uwb_sub = nh.subscribe("/odometry/uwb",2,callback);
//  ros::Subscriber odom_sub = nh.subscribe("/odom",1,odomCallback);
  //发布新的定位
  pub_ = nh.advertise<nav_msgs::Odometry>("/odom/uwb",2);
  spinner.spin();
  //  ros::spin();
  return 0;
}
