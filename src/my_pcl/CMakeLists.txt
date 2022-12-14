cmake_minimum_required(VERSION 2.8.3)
project(my_pcl)
add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  tf
)

find_package(PCL 1.7 REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu
#  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/my_pcl_node.cpp src/My_Filter.cpp)
add_dependencies(${PROJECT_NAME}_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node 
  ${catkin_LIBRARIES}
)


install( TARGETS 
${PROJECT_NAME}_node 
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
