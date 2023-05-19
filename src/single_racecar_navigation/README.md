# BUILD MAP(Gmapping)

roslaunch single_racecar_navigation smart_gazebo.launch

roslaunch single_racecar_navigation slam_gmapping.launch	(new terminal)

rviz	(new terminal)

> note:
>
> edit worldfile in smart_gazebo.launch
>
> add "map" in rviz

rosrun map_server map_saver -f filename
