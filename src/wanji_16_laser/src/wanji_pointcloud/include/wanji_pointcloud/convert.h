/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Wanji 3D LIDAR packets to PointCloud2.

*/

#ifndef _WANJI_POINTCLOUD_CONVERT_H_
#define _WANJI_POINTCLOUD_CONVERT_H_ 1

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <wanji_pointcloud/rawdata.h>

#include <dynamic_reconfigure/server.h>
#include <pcl_ros/point_cloud.h>

namespace wanji_pointcloud
{
  class Convert
  {
  public:

    Convert(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~Convert() {}
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  private:
    
    void processScan(const wanji_msgs::WanjiScan::ConstPtr &scanMsg);
    int getCalibrationUpdateFlag(void);

    ///Pointer to dynamic reconfigure service srv_
    
    boost::shared_ptr<wanji_rawdata::RawData> data_;
    ros::Subscriber wanji_scan_;
    ros::Publisher output_;
    ros::Publisher output_8;
    ros::Publisher output_8L_[8];
    PointCloud::Ptr outMsg_2;
    /// configuration parameters
    typedef struct {
      int npackets;                    ///< number of packets to combine
    } Config;
    Config config_;
    ros::NodeHandle priv_nh_;
  };

} // namespace wanji_pointcloud

#endif // _WANJI_POINTCLOUD_CONVERT_H_
