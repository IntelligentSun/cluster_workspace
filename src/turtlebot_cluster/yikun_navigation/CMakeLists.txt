cmake_minimum_required(VERSION 2.8.3)
project(yikun_navigation)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  yikun_cluster_msgs
  tf
  image_geometry
  image_transport
  message_filters
  roslint
  tf2
  tf2_geometry_msgs
  tf2_ros
  xmlrpcpp
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

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ekf
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${MYSQL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${TINKERFORGE_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})

#########lib##########
###filter######EKF滤波器
add_library(filter_utilities src/filter/filter_utilities.cpp)
add_library(filter_base src/filter/filter_base.cpp)
add_library(ekf src/filter/ekf.cpp)
add_library(ros_filter_utilities src/filter/ros_filter_utilities.cpp)
add_library(ros_filter src/filter/ros_filter.cpp)
add_dependencies(filter_base ${PROJECT_NAME}_gencpp)
target_link_libraries(ros_filter_utilities ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(filter_utilities ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(filter_base filter_utilities ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(ekf filter_base ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(ros_filter ekf ros_filter_utilities ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(ekf_localization_node src/filter/ekf_localization_node.cpp)
target_link_libraries(ekf_localization_node ros_filter ekf ${catkin_LIBRARIES})

##############地图##################
add_library(image_loader src/mapping/image_loader.cpp)
target_link_libraries(image_loader SDL SDL_image ${Boost_LIBRARIES})

add_executable(map_saver src/mapping/map_saver.cpp)
target_link_libraries(map_saver image_loader yaml-cpp ${catkin_LIBRARIES})
#加载地图
add_executable(map_server src/mapping/map_server.cpp)
target_link_libraries(map_server image_loader yaml-cpp ${catkin_LIBRARIES})
#创建空白地图
add_executable(blank_map_generator src/mapping/blank_map_generator.cpp)
target_link_libraries(blank_map_generator ${catkin_LIBRARIES})

###########导航###################
#路径跟踪
add_executable(path_follower_node
  src/local_planner/path_follower_node.cpp
  src/local_planner/path_follower.cpp)
target_link_libraries(path_follower_node
  ${catkin_LIBRARIES}
)
#UWB定位TF
add_executable(transmission src/local_planner/transmission.cpp)
target_link_libraries(transmission ${catkin_LIBRARIES})
