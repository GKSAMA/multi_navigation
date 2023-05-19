cmake_minimum_required(VERSION 3.0.2)
project(smooth_global_planner)

find_package(catkin REQUIRED
  COMPONENTS
    angles
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
    tf2_geometry_msgs
    tf2_ros
    osqp
)

find_package(PythonLibs REQUIRED)
find_package(Protobuf 3.0.0 REQUIRED)
set(GLOG_LIBRARY glog)
find_package(yaml-cpp REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/SmoothGlobalPlanner.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
    tf2_ros
    osqp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
)

add_library(smooth_utils
  src/utils/utils.cpp
)

add_library(${PROJECT_NAME}
  src/quadratic_calculator.cpp
  src/dijkstra.cpp
  src/astar.cpp
  src/grid_path.cpp
  src/gradient_path.cpp
  src/orientation_filter.cpp
  src/planner_core.cpp
  src/path_smoother.cpp
  src/interface/osqp_interface.cpp
  src/interface/sqp_osqp_interface.cpp
  src/smoother_config.cc
  src/smoother_config_loader.cpp
)

add_dependencies(smooth_utils ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GLOG_LIBRARY} ${PROTOBUF_LIBRARY} ${YAML_CPP_LIBRARIES} smooth_utils)

# add_executable(planner
#   src/plan_node.cpp
# )
# add_dependencies(planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(planner
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
# )

add_executable(test_smooth test/path_smoother_test.cpp)
add_dependencies(test_smooth ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_smooth
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PYTHON_LIBRARIES}
  smooth_utils
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# install(TARGETS planner
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)

install(FILES bgp_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
