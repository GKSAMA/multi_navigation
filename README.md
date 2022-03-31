# multi_turtlebot3_navigation

## build

catkin_make  
source devel/setup.bash  

## Prepare

将src/multi_turtlebot3_navigation/lib下的动态库copy至 /usr/lib/x86_64-linux-gnu/gazebo-9/plugins/目录下

## 运行：

roslaunch multi_turtlebot3_navigation multi_turtlebot3_gazebo.launch
roslaunch multi_turtlebot3_navigation move_base_three.launch
roslaunch multi_turtlebot3_navigation navigation_three.launch
(optional)
rosrun multi_turtlebot3_navigation multi_showpath
rosrun multi_turtlebot3_navigation navigation_by_waypoints
(data record)
rosrun multi_turtlebot3_navigation datarecord
(standard show)  
rosrun multi_turtlebot3_navigation showdata
