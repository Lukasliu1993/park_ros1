cmake_minimum_required(VERSION 2.8.3)
project(gps_driver)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  nav_msgs
  tf
  tf2
)

catkin_package(
   INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(gps_driver src/gps_driver.cpp)
add_dependencies(gps_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_driver
  ${catkin_LIBRARIES}
)


