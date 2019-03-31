cmake_minimum_required(VERSION 2.8.3)
project(yikun_common)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  yikun_cluster_msgs
  tf
  image_geometry
  image_transport
  serial
)
find_package(Boost REQUIRED
  COMPONENTS
  filesystem
  program_options
  signals
  system
  thread
)

#####mysql###########
set(MYSQL_INCLUDE_DIRS /usr/include/mysql)
set(MYSQL_LIBRARY_DIRS /usr/lib/x86_64-linux-gnu -lmysqlclient -lpthread -lz -lm -ldl)

#########eigen3###########
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES db_helper
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${MYSQL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${TINKERFORGE_INCLUDE_DIRS})

#########lib##########
#数据库操作接口
add_library(db_helper src/mysql/db_helper.cpp)
add_dependencies(db_helper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} yikun_common_gencpp)
target_link_libraries(db_helper ${MYSQL_LIBRARY_DIRS} ${catkin_LIBRARIES})
add_executable(mysql_test
  src/mysql/mysql_test.cpp)
target_link_libraries(mysql_test ${catkin_LIBRARIES})
###########slave manager##########
#系统命令接口
add_library(system_helper src/tools/system_helper.cpp)
add_dependencies(system_helper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} yikun_common_gencpp)
target_link_libraries(system_helper ${catkin_LIBRARIES})
###########################
#数据库同步
add_executable(robot_info_sync
  src/cluster_manager/robot_info_sync.cpp
  src/cluster_manager/robot_info.cpp
  src/cluster_manager/robot_thread.cpp)
target_link_libraries(robot_info_sync system_helper db_helper ${catkin_LIBRARIES})
#############
#机器人位置更新
add_executable(robot_pose src/robot_manager/robot_pose.cpp)
target_link_libraries(robot_pose db_helper ${catkin_LIBRARIES})
#Turtlebot在世界坐标系下的相对位置转换
add_executable(tf_viewer src/robot_manager/tf_viewer.cpp)
target_link_libraries(tf_viewer ${catkin_LIBRARIES})
############
#服务
add_executable(service_node
  src/service/service_node.cpp
  src/service/service.cpp)
target_link_libraries(service_node ${catkin_LIBRARIES})
############
#服务调用Demo
add_executable(cluster_demo
  src/service/service_client.cpp
  src/service/cluster_demo.cpp
  src/service/publisher.cpp)
target_link_libraries(cluster_demo ${catkin_LIBRARIES})

###########
#UWB数据采集
add_executable(uwb_node
  src/uwb/uwb.cpp
  src/uwb/uwb_node.cpp
  src/service/publisher.cpp)
target_link_libraries(uwb_node ${catkin_LIBRARIES})
############
#路径解析调用Demo
add_executable(path_finder
  src/path_finder/path_finder.cpp
  src/service/service_client.cpp
  src/service/publisher.cpp)
target_link_libraries(path_finder ${catkin_LIBRARIES})
