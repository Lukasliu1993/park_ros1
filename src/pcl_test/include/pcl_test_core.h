#pragma once

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h> 
#include <laser_geometry/laser_geometry.h>
#include "geometry_msgs/TransformStamped.h"
#include <tf2_eigen/tf2_eigen.h>
// using laser_geometry::LaserProjection;
class PclTestCore
{

  private:
    ros::Subscriber sub_point_cloud_;

    ros::Publisher pub_filtered_points_;

    void point_cb(const sensor_msgs::PointCloud2ConstPtr& in_cloud);
    float resolution_;
    std::string input_topic_, output_topic_;

  public:
    PclTestCore();
    ~PclTestCore();
    void Spin();
    void run();
};