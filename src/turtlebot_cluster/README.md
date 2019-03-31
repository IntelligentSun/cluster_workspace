# turtlebot_cluster

Turtlebot+UWB 集群控制软件包 

1. __环境搭建__
  
  * __系统安装__
  
  系统版本为 _Ubuntu 16.04_,直接下载ISO可能很慢，可以先下载BitTorrent，下载好镜像后制作系统安装盘，安装系统。镜像下载地址: _http://releases.ubuntu.com/xenial/_
  
  * __ROS安装__
  
  ROS版本为 _Kinetic_,具体安装教程见 : _http://wiki.ros.org/kinetic/Installation/Ubuntu_
    
    $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    $ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    $ sudo apt-get update
    $ sudo apt-get install ros-kinetic-desktop-full
    $ sudo rosdep init
    $ rosdep update
    $ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
    $ source ~/.bashrc
    $ sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
  
  * __工作空间创建__
    
    ROS详细教程(使用前请务必阅读相关基础教程) : _http://wiki.ros.org/ROS/Tutorials_
      
      * $ `mkdir -p ~/ros_ws/src`
      * $ `cd ~/ros_ws/src`
      * $ `catkin_make`
    
2. __所需的依赖包__

  * __kobuki（Kobuki Driver）__
    
    $ `sudo apt-get install ros-kinetic-kobuki*`
    
  * __turtlebot（The turtlebot stack provides all the basic drivers for running and using a TurtleBot）__
    
    $ `git clone https://github.com/ShanSuIntelligent/turtlebot.git`
  
  * __turtlebot_apps(A group of simple demos and exmaples to run on your TurtleBot to help you get started with ROS and TurtleBot.)__
    
    $ `git clone https://github.com/ShanSuIntelligent/turtlebot_apps.git`
    
  * __Turtlebot 教程__
    ros wiki : http://wiki.ros.org/turtlebot/Tutorials/indigo
    turtlebot : https://www.turtlebot.com/
    
3. __UWB编辑路径导航Demo__
  
  * __Turtlebot__
    
    * $ `cd ~/ros_ws`
    * $ `catkin_make`
    * $ `source devel/setup.bash`
    * $ `roslaunch turtlebot_bringup minimal.launch`
    * $ `roslaunch yikun_navigation navigation.launch`

  * __上位机__
    
    * $ `cd ~/ros_ws`
    * $ `catkin_make`
    * $ `source devel/setup.bash`
    * $ `rosrun yikun_common uwb_node`
    * $ `roslaunch yikun_common common.launch`
  
