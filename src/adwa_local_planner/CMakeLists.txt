cmake_minimum_required(VERSION 3.0.2)
project(adwa_local_planner)
#include_directories(SYSTEM "/foo/bar")
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
        COMPONENTS
        angles
        base_local_planner
        cmake_modules
        costmap_2d
        dynamic_reconfigure
        nav_core
        nav_msgs
        pluginlib
        sensor_msgs
        pcl_conversions
        roscpp
        tf
        tf2
        tf2_geometry_msgs
        tf2_ros
        my_fuzzy_controller
        )

find_package(Eigen3 REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

link_directories(${catkin_LIBRARIES})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
        cfg/ADWAPlanner.cfg
)

add_library(${PROJECT_NAME} 
        src/ackermann_odometry_helper.cpp
        src/adwa_trajectory_sample_generator.cpp
        src/adwa_planner_utils.cpp
        src/adwa_scored_sampling_planner.cpp
        src/desired_direction_cost.cpp
        src/vertical_distance_cost.cpp
        src/min_turn_radius_cost.cpp
        src/heading_cost_function.cpp
        src/adwa_planner.cpp 
        src/adwa_planner_ros.cpp)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        base_local_planner
        dynamic_reconfigure
        nav_msgs
        pluginlib
        sensor_msgs
        roscpp
        tf2
        tf2_ros
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS adwa_local_planner
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

install(FILES cfg/adwa_planner_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

#install(DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#        PATTERN ".svn" EXCLUDE
#        )
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )

add_executable(tftest src/tftest.cpp)
add_dependencies(tftest ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(tftest ${catkin_LIBRARIES} ${PROJECT_NAME})
