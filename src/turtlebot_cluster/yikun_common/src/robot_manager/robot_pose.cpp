/*
 *  robot_pose.cpp
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "yikun_common/mysql/db_helper.h"

#define PI 3.1415926


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "yikun_robot_pose");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  //数据库连接
  yikun_common::DbHelper irdb("127.0.0.1","yk");
  while(!irdb.dbconnect()){
    ROS_ERROR("Unable to connect to irdb");
    sleep(1);
  }

  // configuring parameters
  std::string map_frame, base_frame;
  double publish_frequency;
  bool is_stamped;
  ros::Publisher p_pub;
  //世界坐标
  nh_priv.param("map_frame",map_frame,std::string("/map"));
  //小车坐标
	nh_priv.param("base_frame",base_frame,std::string("/base_link"));
  //更新频率
	nh_priv.param("publish_frequency",publish_frequency,10.0);
  //时间戳
	nh_priv.param("is_stamped", is_stamped, true);

  if(is_stamped)
    p_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_pose", 1);
  else
    p_pub = nh.advertise<geometry_msgs::Pose>("robot_pose", 1);

  // create the listener
  tf::TransformListener listener;
  listener.waitForTransform(map_frame, base_frame, ros::Time(), ros::Duration(10.0));

  ros::Rate rate(publish_frequency);
  while (nh.ok())
  {
    tf::StampedTransform transform;
    try
    {
      //计算小车坐标到世界坐标之间的坐标转换
      listener.lookupTransform(map_frame, base_frame, ros::Time(0), transform);
      // construct a pose message
      //robot pose
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = map_frame;
      pose_stamped.header.stamp = ros::Time::now();

      pose_stamped.pose.orientation.x = transform.getRotation().getX();
      pose_stamped.pose.orientation.y = transform.getRotation().getY();
      pose_stamped.pose.orientation.z = transform.getRotation().getZ();
      pose_stamped.pose.orientation.w = transform.getRotation().getW();

      pose_stamped.pose.position.x = transform.getOrigin().getX();
      pose_stamped.pose.position.y = transform.getOrigin().getY();
      pose_stamped.pose.position.z = transform.getOrigin().getZ();

      double roll,pitch,yaw;
      transform.getBasis().getRPY(roll,pitch,yaw);
      yaw = yaw*180/PI;
			
      //更新到数据库
      if(irdb.updatePosToRuntimeInfo(transform.getOrigin().getX(),transform.getOrigin().getY(),yaw))
      {
        if(is_stamped)
          p_pub.publish(pose_stamped);
        else
          p_pub.publish(pose_stamped.pose);
      }
      else{
        ROS_WARN("Robot Pose never update!!!");
      }
      //
    }
    catch (tf::TransformException &ex)
    {
      // just continue on
    }
    rate.sleep();
  }
  return 0;
}
