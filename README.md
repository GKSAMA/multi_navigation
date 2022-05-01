# multi_turtlebot3_navigation

## build

catkin_make  
source devel/setup.bash  

## Prepare

1. 将src/multi_turtlebot3_navigation/lib下的动态库copy至 /usr/lib/x86_64-linux-gnu/gazebo-9/plugins/目录下

<<<<<<< HEAD
2. 需要对每个终端添加变量 `TURTLEBOT3_MODEL=burger`
或者直接在.bashrc中添加 `export TURTLEBOT3_MODEL=burger`

3. 在src下运行git clone https://github.com/locusrobotics/robot_navigation.git 以使用dwb_local_planner

## 运行(turtlebot)：
=======
需要对每个终端添加变量 `TURTLEBOT3_MODEL=burger`
或者直接在.bashrc中添加 `export TURTLEBOT3_MODEL=burger`

## 运行：
>>>>>>> be26252513f0a73642d7d5227160c4e545be73fa

roslaunch multi_turtlebot3_navigation multi_turtlebot3_gazebo.launch  
roslaunch multi_turtlebot3_navigation move_base_tree.launch  
roslaunch multi_turtlebot3_navigation navigation_tree.launch  
(optional)  
rosrun multi_turtlebot3_navigation multi_showpath  
rosrun multi_turtlebot3_navigation navigation_by_waypoints  
(data record)  
rosrun multi_turtlebot3_navigation datarecord  
(standard show)  
rosrun multi_turtlebot3_navigation showdata  

