/*
 *  tf_viewer.cpp
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "message_filters/subscriber.h"
#include "yikun_cluster_msgs/UWB.h"

std::map<int,std::string> links;
void callback(const yikun_cluster_msgs::UWB::ConstPtr& msg)
{
  static tf::TransformBroadcaster br;
  tf::Transform tmp_tf(tf::createQuaternionFromYaw(0.0),
                       tf::Vector3(msg->x,
                                   msg->y,
                                   msg->z));
  ros::Time transform_time = msg->header.stamp;
  std::string frame_id;
  //<tag id,turtlebot link>
  std::map<int,std::string>::iterator it;
  it = links.find(msg->tag);
  if(it != links.end())
  {
    frame_id = it->second;
    br.sendTransform(tf::StampedTransform(tmp_tf, transform_time, msg->header.frame_id, frame_id));
  }

}

int main(int argc,char* argv[])
{
  ros::init(argc,argv,"tf_viewer");
  ros::NodeHandle nh;
  //Initial
  links[14] = "turtlebot_01_link";  //tag  14
  links[0] = "turtlebot_02_link";   //tag  0
  links[25] = "turtlebot_03_link";  //tag  25
  links[7] = "turtlebot_04_link";   //tag  7
  links[1] = "turtlebot_05_link";   //tag  1
  //监听UWB数据
  ros::Subscriber sub = nh.subscribe("/uwb_pose",10,callback);
  ros::spin();
  return 0;
}
