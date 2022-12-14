cmake_minimum_required(VERSION 2.8.3)
project(loop_path_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    costmap_2d
    nav_core
    pluginlib
    roscpp
    )

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(loop_path_planner src/loop_path_planner.cpp)
target_link_libraries(loop_path_planner
    ${catkin_LIBRARIES}
    )

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)    

install(FILES global_planner_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
