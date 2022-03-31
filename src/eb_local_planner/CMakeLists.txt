cmake_minimum_required(VERSION 3.0.2)
project(eb_local_planner)

find_package(catkin REQUIRED
        COMPONENTS
        base_local_planner
        control_toolbox
        costmap_2d
        dynamic_reconfigure
        geometry_msgs
        nav_core
        nav_msgs
        pluginlib
        roscpp
        tf2_eigen
        tf2_geometry_msgs
        tf2_ros
        )
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(Eigen3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()

generate_dynamic_reconfigure_options(cfg/EBandPlanner.cfg)

catkin_package(
        CATKIN_DEPENDS
        base_local_planner
        control_toolbox
        costmap_2d
        geometry_msgs
        nav_core
        nav_msgs
        pluginlib
        roscpp
        tf2_eigen
        tf2_geometry_msgs
        tf2_ros
        dynamic_reconfigure
        INCLUDE_DIRS include
        LIBRARIES eband_local_planner
)

include_directories(
        include
        ${Eigen3_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

add_library(eb_local_planner
        src/conversions_and_types.cpp
        src/eb_local_planner.cpp
        src/eb_local_planner_ros.cpp
        src/eb_trajectory_controller.cpp
        src/eb_visualization.cpp)
add_dependencies(eb_local_planner ${eb_local_planner_EXPORTED_TARGETS})
target_link_libraries(eb_local_planner ${catkin_LIBRARIES})

install(
        DIRECTORY include/eb_local_planner/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
        TARGETS eb_local_planner
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
        FILES elp_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)