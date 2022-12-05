cmake_minimum_required(VERSION 2.8.3)

project(my_fuzzy_controller)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES my_fuzzy_controller
)

include_directories(
    include
)

add_library(${PROJECT_NAME}
    src/my_fuzzy_controller.cpp
)

add_executable(test2 test/test2.cpp)
# add_dependencies(test2 ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(test2 ${PROJECT_NAME})