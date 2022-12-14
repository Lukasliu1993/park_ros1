cmake_minimum_required(VERSION 2.8.3)
project(wanji_pointcloud)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        angles
        nodelet
        pcl_ros
        roscpp
        sensor_msgs
        tf
        wanji_msgs
        dynamic_reconfigure
        pcl_conversions)

find_package(Boost COMPONENTS signals)
find_package(PkgConfig REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES wanji_pointcloud
        CATKIN_DEPENDS
        angles
        nodelet
        pcl_ros
        roscpp
        sensor_msgs
        tf
        wanji_driver
        wanji_msgs
        dynamic_reconfigure
        pcl_conversions
        DEPENDS
        Boost
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
)

add_library(wanji_data src/rawdata.cc)
target_link_libraries(wanji_data
        ${catkin_LIBRARIES}
        ${libpcap_LIBRARIES})

add_library(wanji_point src/convert.cc)
target_link_libraries(wanji_point
        wanji_data
        ${catkin_LIBRARIES})


add_library(wanji_pointcloud_nodelet src/cloud_nodelet.cc)
target_link_libraries(wanji_pointcloud_nodelet wanji_point
        ${catkin_LIBRARIES})

add_executable(wanji_pointcloud_node src/cloud_node.cc)
target_link_libraries(wanji_pointcloud_node
        wanji_point
        ${catkin_LIBRARIES}
        ${libpcap_LIBRARIES})
add_dependencies(wanji_pointcloud_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )


if(catkin_EXPORTED_TARGETS)
    add_dependencies(wanji_data ${catkin_EXPORTED_TARGETS})
endif()


