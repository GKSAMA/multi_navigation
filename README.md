# multi_turtlebot3_navigation

## Environment

This project has built successfully in Ubuntu18.04 and Ubuntu20.04.

Please install ROS first. Follow this [page](wiki.ros.org/melodic/Installation) to install ROS.

## dependencies

* sudo apt install ros-${ros_version}-base-local-planner
* sudo apt install ros-${ros_version}-move-base
* sudo apt install ros-${ros_version}-turtlebot3

## build

git clone https://github.com/GKSAMA/multi_navigation.git

cd ..

catkin_make

source devel/setup.bash

## Prepare

1. 将src/multi_turtlebot3_navigation/lib下的动态库copy至 /usr/lib/x86_64-linux-gnu/gazebo-9/plugins/目录下
2. 需要对每个终端添加变量 `TURTLEBOT3_MODEL=burger`  `或者直接在.bashrc中添加 `export TURTLEBOT3_MODEL=burger`
3. 在src下运行git clone https://github.com/locusrobotics/robot_navigation.git 以使用dwb_local_planner

## 运行(turtlebot)：

1. roslaunch multi_turtlebot3_navigation multi_turtlebot3_gazebo.launch
2. roslaunch multi_turtlebot3_navigation move_base_tree.launch
3. roslaunch multi_turtlebot3_navigation navigation_tree.launch



(optional)

rosrun multi_turtlebot3_navigation multi_showpath

rosrun multi_turtlebot3_navigation navigation_by_waypoints

(data record)

rosrun multi_turtlebot3_navigation datarecord

(data analysis)

rosrun multi_turtlebot3_navigation showdata
