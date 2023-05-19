cmake_minimum_required(VERSION 2.8.3)
project(single_racecar_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  tf
  actionlib
  actionlib_msgs
  std_msgs
  message_generation
  multi_turtlebot3_navigation
  parking_task
  gazebo_ros
  hybrid_a_star
  Astar
  )

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   ParkingSpaceInfo.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES teleop_nps
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
   CATKIN_DEPENDS
   roscpp
   rospy
   std_msgs
   actionlib
   message_runtime
   multi_turtlebot3_navigation
   parking_task
   hybrid_a_star
   Astar
   )

find_package(gazebo REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${GAZEBO_INCLUDE_DIRS}
 ${Qt5Core_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIR}
 )

add_library(gazebo_manager_controller
   src/gazebo_controller_manager.cpp
)

add_library(single_racecar_utils
   src/single_racecar_navigation/utils.cpp
   src/single_racecar_navigation/load_parameters.cpp
)

target_link_libraries(single_racecar_utils ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES})

add_executable(run src/run.cpp)
target_link_libraries(run ${catkin_LIBRARIES})

add_executable(showpath src/draw_line.cpp)
target_link_libraries(showpath ${catkin_LIBRARIES} multi_nav_utils)

add_executable(datarecordsmart src/data_record.cpp)
target_link_libraries(datarecordsmart ${catkin_LIBRARIES} multi_nav_utils)

add_executable(task src/task.cpp)
target_link_libraries(task 
   ${catkin_LIBRARIES} 
   parking_task 
   gazebo_manager_controller 
   single_racecar_utils)

add_executable(task2 src/task2.cpp)
target_link_libraries(task2 
   ${catkin_LIBRARIES} 
   parking_task 
   gazebo_manager_controller 
   single_racecar_utils)

add_executable(task3 src/task3.cpp)
target_link_libraries(task3 
   ${catkin_LIBRARIES} 
   parking_task 
   gazebo_manager_controller
   single_racecar_utils
   hybrid_a_star
   multi_nav_utils
   )

add_executable(test src/single_racecar_navigation/test.cpp)
target_link_libraries(test 
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   Astar)

add_executable(bdwatest src/single_racecar_navigation/bdwatest.cpp)
target_link_libraries(bdwatest  ${catkin_LIBRARIES} single_racecar_utils)

add_library(DrawTrajectoryPlugin SHARED src/gazebo_plugin/draw_trajectory_plugin.cc)
target_link_libraries(DrawTrajectoryPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})