/*
 *  uwb.h
 *  Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#ifndef UWB_H
#define UWB_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <math.h>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/hex.hpp>
#include <boost/regex.hpp>
#include <Eigen/Eigen>
#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "time.h"

#define TRILATERATION   (1)

#define REGRESSION_NUM  (10)
#define SPEED_OF_LIGHT  (299702547.0)   // in m/s in air
#define NUM_ANCHORS     (5)
#define REF_ANCHOR      (5)	//anchor IDs are 1,2,3,4,5 etc. (don't start from 0!)

#define		TRIL_3SPHERES 3
#define		TRIL_4SPHERES 4

/* Largest nonnegative number still considered zero */
#define   MAXZERO  0.001

#define		ERR_TRIL_CONCENTRIC               -1
#define		ERR_TRIL_COLINEAR_2SOLUTIONS			-2
#define		ERR_TRIL_SQRTNEGNUMB              -3
#define		ERR_TRIL_NOINTERSECTION_SPHERE4   -4
#define		ERR_TRIL_NEEDMORESPHERE           -5

#ifndef MAX_SIZE
#define MAX_SIZE 1024
#endif
//位置点定义
typedef struct vec3d	vec3d;
struct vec3d {
  double	x;
  double	y;
  double	z;
};
namespace yikun_common {
//基站位置
typedef struct anchorarray {
  Eigen::Vector3f A0; //{x, y, z}
  Eigen::Vector3f A1;
  Eigen::Vector3f A2;
  Eigen::Vector3f A3;
}AnchorArray;
//UWB定位信息
typedef struct uwb_info {
  int tag_id; //标签号
  vec3d position; //位置
}UWBInfo;

class UWB
{
public:
  typedef boost::shared_ptr<yikun_common::UWB> Ptr;
  UWB(const AnchorArray anchorArray,const std::string port,
      const unsigned int baudrate,const unsigned int timeout);
  ~UWB();
  /**
   * @brief 连接串口设备
   */
  bool connect();
  /**
   * @brief 初始化
   */
  bool prepare();
  /**
   * @brief 通讯测试
   * @param recv 接收信息
   */
  bool communicate(std::string &recv);
  /**
   * @brief 获取并解析UWB系统当前发布的数据
   * @param info 当前标签队列的定位信息
   */
  bool getNewData(std::vector<UWBInfo>& info);

  /* Return the difference of two vectors, (vector1 - vector2). */
  vec3d vdiff(const vec3d vector1, const vec3d vector2);

  /* Return the sum of two vectors. */
  vec3d vsum(const vec3d vector1, const vec3d vector2);

  /* Multiply vector by a number. */
  vec3d vmul(const vec3d vector, const double n);

  /* Divide vector by a number. */
  vec3d vdiv(const vec3d vector, const double n);

  /* Return the Euclidean norm. */
  double vdist(const vec3d v1, const vec3d v2);

  /* Return the Euclidean norm. */
  double vnorm(const vec3d vector);

  /* Return the dot product of two vectors. */
  double dot(const vec3d vector1, const vec3d vector2);

  /* Replace vector with its cross product with another vector. */
  vec3d cross(const vec3d vector1, const vec3d vector2);

  /* Return the GDOP (Geometric Dilution of Precision) rate between 0-1.
   * Lower GDOP rate means better precision of intersection.
   */
  double gdoprate(const vec3d tag, const vec3d p1, const vec3d p2, const vec3d p3);

  /* Intersecting a sphere sc with radius of r, with a line p1-p2.
   * Return zero if successful, negative error otherwise.
   * mu1 & mu2 are constant to find points of intersection.
  */
  int sphereline(const vec3d p1, const vec3d p2, const vec3d sc, double r, double *const mu1, double *const mu2);

  /* Return TRIL_3SPHERES if it is performed using 3 spheres and return
   * TRIL_4SPHERES if it is performed using 4 spheres
   * For TRIL_3SPHERES, there are two solutions: result1 and result2
   * For TRIL_4SPHERES, there is only one solution: best_solution
   *
   * Return negative number for other errors
   *
   * To force the function to work with only 3 spheres, provide a duplicate of
   * any sphere at any place among p1, p2, p3 or p4.
   *
   * The last parameter is the largest nonnegative number considered zero;
   * it is somewhat analogous to machine epsilon (but inclusive).
  */
  int trilateration(vec3d *const result1,
            vec3d *const result2,
            vec3d *const best_solution,
            const vec3d p1, const double r1,
                    const vec3d p2, const double r2,
                    const vec3d p3, const double r3,
                    const vec3d p4, const double r4,
                    const double maxzero);


  /* This function calls trilateration to get the best solution.
   *
   * If any three spheres does not produce valid solution,
   * then each distance is increased to ensure intersection to happens.
   *
   * Return the selected trilateration mode between TRIL_3SPHERES or TRIL_4SPHERES
   * For TRIL_3SPHERES, there are two solutions: solution1 and solution2
   * For TRIL_4SPHERES, there is only one solution: best_solution
   *
   * nosolution_count = the number of failed attempt before intersection is found
   * by increasing the sphere diameter.
  */
  int deca_3dlocate (	vec3d	*const solution1,
            vec3d	*const solution2,
            vec3d	*const best_solution,
            int		*const nosolution_count,
            double	*const best_3derror,
            double	*const best_gdoprate,
            vec3d p1, double r1,
            vec3d p2, double r2,
            vec3d p3, double r3,
            vec3d p4, double r4,
            int *combination);
  /**
   * @brief 标签位置解算
   * @param best_solution 标签位置解算结果
   * @param use4thAnchor  是否使用四个基站
   * @param anchorArray   各个基站的位置
   * @param distanceArray 标签到各个基站的距离
   */
  int GetLocation(vec3d *best_solution, int use4thAnchor, vec3d* anchorArray, int *distanceArray);

private:
  std::string port_;
  serial::Serial serial_;
  unsigned int baudrate_;
  unsigned int timeout_;
  vec3d anchorArray_[4];
};

}//namespace

#endif // UWB_H
