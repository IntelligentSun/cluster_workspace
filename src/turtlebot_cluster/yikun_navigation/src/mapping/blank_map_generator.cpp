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

#ifndef GRID_VAL
#define GRID_VAL 254
#endif

using namespace std;
typedef struct blankmap {
  std::string name;   //地图名
  int width;          //像素尺寸
  int height;
  float resolution;   //地图分辨率，实际尺寸 = 像素尺寸×分辨率
}BlankMap;

class BlankMapGenerator
{
  public:
    BlankMapGenerator(const BlankMap& map)
      : saved_map_(false),
        map_(map)
    {
      ros::NodeHandle n;
      this->generate();
    }
    void generate()
    {
      ROS_INFO("Generate a %d X %d map @ %.3f m/pix",
               map_.width,
               map_.height,
               map_.resolution);
      std::string mapdatafile = map_.name + ".pgm";
      map_path_png_ = map_.name + ".png";
      ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
      FILE* out = fopen(mapdatafile.c_str(), "w");
      if (!out)
      {
        ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
        return;
      }

      fprintf(out, "P5\n# CREATOR: balnk_map_generator.cpp %.3f m/pix\n%d %d\n255\n",
              map_.resolution, map_.width, map_.height);
      //图片初始化
      cv::Mat imageData = cv::Mat::zeros(map_.height,map_.width,CV_8UC3);
      for(unsigned int y = 0; y < map_.height; y++) {
        for(unsigned int x = 0; x < map_.width; x++) {
          unsigned int i = x + (map_.height - y - 1) * map_.width;
          // determine the index into the image data array
          int index = (x + (y * map_.width)) * 3;
          // r
          imageData.data[index] = GRID_VAL;
          // g
          imageData.data[++index] = GRID_VAL;
          // b
          imageData.data[++index] = GRID_VAL;
          fputc(GRID_VAL, out);
        }
      }
      //保存
      cv::imwrite(map_path_png_.c_str(),imageData);
      fclose(out);

      std::string mapmetadatafile = map_.name + ".yaml";
//      std::string webmapfile = map_.name + ".txt";
      ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
      FILE* yaml = fopen(mapmetadatafile.c_str(), "w");
//      FILE* txt = fopen(webmapfile.c_str(), "w");

      double origin_x,origin_y;
      origin_x = 0;
      origin_y = 0;
      double yaw = 0;
//      origin_x = -(map_.width*map_.resolution/2.0);
//      origin_y = -(map_.height*map_.resolution/2.0);
      //图片属性
      fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
              mapdatafile.c_str(), map_.resolution, origin_x, origin_y, yaw);
      fclose(yaml);
      
      //共享图片的uuid
//      std::string uuid = boost::uuids::to_string(boost::uuids::random_generator()());
//      fprintf(txt, "name: %s\nresolution: %f\norigin_x: %f\norigin_y: %f\norigin_theta: %f\nwidth: %d\nheight: %d\nuuid: %s",
//              map_.name.c_str(),map_.resolution,origin_x,origin_y, yaw*M_PI/180,
//              map_.width,map_.height,uuid.c_str());
//      fclose(txt);

      ROS_INFO("Done\n");
      saved_map_ = true;
    }

    ros::Subscriber map_sub_;
    bool saved_map_;
private:
    std::string map_path_png_;
    BlankMap map_;

};

#define USAGE "Usage: \n" \
              "  blank_map_genetator <map_name> <witdh(pix)> <height(pix)> <resolution(m)> \n"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "map_generator");
  std::string mapname = "map";
  BlankMap map;
  if(argc < 4){
    puts(USAGE);
    return 0;
  } else {
    map.name = argv[1];
    map.width = atoi(argv[2]);
    map.height = atoi(argv[3]);
    map.resolution = atof(argv[4]);
  }

  if(*map.name.rbegin() == '/')
    map.name += "map";

  BlankMapGenerator mg(map);

  while(!mg.saved_map_ && ros::ok())
    ros::spinOnce();

  return 0;
}
