cmake_minimum_required(VERSION 2.8.3)
project(wanji_driver)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Debug)#RelWithDebInfo
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(${PROJECT_NAME}_CATKIN_DEPS
    angles
    pcl_ros
    roscpp
    sensor_msgs
    tf
    dynamic_reconfigure
    wanji_msgs
    nodelet
)

set(libpcap_LIBRARIES -lpcap)

find_package(catkin REQUIRED COMPONENTS
	${${PROJECT_NAME}_CATKIN_DEPS}
  pcl_conversions
  rospy
  std_msgs
  genmsg
  cv_bridge
  message_generation
  nodelet
  roscpp
)

find_package(Boost COMPONENTS signals)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(PkgConfig REQUIRED)


include_directories(
		include
		${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}
	${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
# Generate dynamic_reconfigure server
generate_dynamic_reconfigure_options(cfg/WanjiNode.cfg)
catkin_package(
	LIBRARIES wanji_driver
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
    CATKIN_DEPENDS message_runtime std_msgs
    )


add_library(wanji_input src/input.cc)
target_link_libraries(wanji_input
		${catkin_LIBRARIES}
		${libpcap_LIBRARIES})

add_library(wanji_driver src/wanji_driver.cpp)
target_link_libraries(wanji_driver
		wanji_input
		${catkin_LIBRARIES})




# build the nodelet version
add_library(wanji_driver_nodelet src/wanji_nodelet.cc src/wanji_driver.cpp)
target_link_libraries(wanji_driver_nodelet
		wanji_input
		${catkin_LIBRARIES}
		)

add_executable(wanji_driver_node src/wanji_driver_node.cpp)

if(catkin_EXPORTED_TARGETS)
	add_dependencies(wanji_input ${catkin_EXPORTED_TARGETS})
endif()

target_link_libraries(wanji_driver_node
		wanji_driver
		wanji_input
		${catkin_LIBRARIES}
		${libpcap_LIBRARIES}
		)
