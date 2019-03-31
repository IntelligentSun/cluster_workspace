/*
 *  map_saver.cpp
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include <cstdio>
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/GetMap.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>

using namespace std;

/**
 * @brief Map generation node.
 */
class MapGenerator
{
  public:
    MapGenerator(const std::string& mapname) : mapname_(mapname), saved_map_(false)
    {
      ros::NodeHandle n;
      ROS_INFO("Waiting for the map");
      map_sub_ = n.subscribe("map", 1, &MapGenerator::mapCallback, this);

    }
    void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
    {
      ROS_INFO("Received a %d X %d map @ %.3f m/pix",
               map->info.width,
               map->info.height,
               map->info.resolution);
      std::string mapdatafile = mapname_ + ".pgm";
      map_path_png_ = mapname_ + ".png";
      ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
      FILE* out = fopen(mapdatafile.c_str(), "w");
      if (!out)
      {
        ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
        return;
      }

      fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
              map->info.resolution, map->info.width, map->info.height);
      cv::Mat imageData = cv::Mat::zeros(map->info.height,map->info.width,CV_8UC3);
      for(unsigned int y = 0; y < map->info.height; y++) {
        for(unsigned int x = 0; x < map->info.width; x++) {
          unsigned int i = x + (map->info.height - y - 1) * map->info.width;
          int val;
          if (map->data[i] == 0) { //occ [0,0.1)
            val = 254;
            fputc(254, out);
          } else if (map->data[i] == +100) { //occ (0.65,1]
            val = 0;
            fputc(000, out);
          } else { //occ [0.1,0.65]
            val = 205;
            fputc(205, out);
          }
          // determine the index into the image data array
          int index = (x + (y * map->info.width)) * 3;
          // r
          imageData.data[index] = val;
          // g
          imageData.data[++index] = val;
          // b
          imageData.data[++index] = val;
        }
      }

      cv::imwrite(map_path_png_.c_str(),imageData);
      fclose(out);

      std::string mapmetadatafile = mapname_ + ".yaml";
      std::string webmapfile = mapname_ + ".txt";
      ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
      FILE* yaml = fopen(mapmetadatafile.c_str(), "w");
      FILE* txt = fopen(webmapfile.c_str(), "w");

      geometry_msgs::Quaternion orientation = map->info.origin.orientation;
      tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
      double yaw, pitch, roll;
      mat.getEulerYPR(yaw, pitch, roll);
      double origin_x,origin_y;
      origin_x = -(map->info.width*map->info.resolution/2.0);
      origin_y = -(map->info.height*map->info.resolution/2.0);
      fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
              mapdatafile.c_str(), map->info.resolution, origin_x, origin_y, yaw);
      fclose(yaml);

      std::string uuid = boost::uuids::to_string(boost::uuids::random_generator()());
      fprintf(txt, "name: %s\nresolution: %f\norigin_x: %f\norigin_y: %f\norigin_theta: %f\nwidth: %d\nheight: %d\nuuid: %s",
              mapname_.c_str(),map->info.resolution,origin_x,origin_y, yaw*M_PI/180,
              map->info.width,map->info.height,uuid.c_str());
      fclose(txt);

      ROS_INFO("Done\n");
      saved_map_ = true;
    }

    std::string mapname_;
    ros::Subscriber map_sub_;
    bool saved_map_;
private:
    std::string map_path_png_;

};

#define USAGE "Usage: \n" \
              "  map_saver -h\n"\
              "  map_saver [-f <mapname>] [ROS remapping args]"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_saver");
  std::string mapname = "map";

  for(int i=1; i<argc; i++)
  {
    if(!strcmp(argv[i], "-h"))
    {
      puts(USAGE);
      return 0;
    }
    else if(!strcmp(argv[i], "-f"))
    {
      if(++i < argc)
        mapname = argv[i];
      else
      {
        puts(USAGE);
        return 1;
      }
    }
    else
    {
      puts(USAGE);
      return 1;
    }
  }

  if(*mapname.rbegin() == '/')
    mapname += "map";

  MapGenerator mg(mapname);

  while(!mg.saved_map_ && ros::ok())
    ros::spinOnce();

  return 0;
}
