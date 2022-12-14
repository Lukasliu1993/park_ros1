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
#include <string>
#include <fstream>
#include <iostream>
#include "wanji_pointcloud/convert.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/point_types.h>
using namespace std;
namespace wanji_pointcloud
{
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new wanji_rawdata::RawData()),priv_nh_(private_nh)
  {
    data_->setup(private_nh);
    outMsg_2  = PointCloud::Ptr(new PointCloud);

    // advertise output point cloud (before subscribing to input data)
    output_ =node.advertise<sensor_msgs::PointCloud2>("wanji_point", 10);
      
    // subscribe to WanjiScan packets
    wanji_scan_ =
      node.subscribe("wanji_packets", 10,
                     &Convert::processScan, (Convert *) this,
                     ros::TransportHints().tcpNoDelay(true));
   
  }
  
  /** @brief get calibration update flag. */
  int Convert::getCalibrationUpdateFlag(void)
  {
    ros::NodeHandle node;//使用公共节点
    int calibrationUpdateFlag=0;
    node.getParam("calibration_update",calibrationUpdateFlag);
    if(calibrationUpdateFlag == 1)
    //   fprintf(stderr,"calibrationUpdateFlag=%d\n",calibrationUpdateFlag);
      node.setParam("calibration_update",0);

    return calibrationUpdateFlag;
  }
   

  /** @brief Callback for raw scan messages. */
  void Convert::processScan(const wanji_msgs::WanjiScan::ConstPtr &scanMsg)
  {
    // allocate a point cloud with same time and frame ID as raw data
    wanji_rawdata::VPointCloud::Ptr
      outMsg(new wanji_rawdata::VPointCloud());
    // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment

    //check whether "calibration_update" was updated
    if(getCalibrationUpdateFlag() == 1)
    {
      //if param updated,reset vertical angle resolution
      data_->setCalibration(priv_nh_);
    }
    
    // process each packet provided by the driver 0.2:120
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    {
      data_->unpack(scanMsg->packets[i], *outMsg);
    }
    //fprintf(stderr,"size:%d\n",scanMsg->packets.size());
    // fprintf(stderr,"angle=%d\n",scanMsg->packets[0].data[3]<<8 | scanMsg->packets[0].data[2]);

	// publish the accumulated cloud message
	ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
					 << " Wanji points, time: " << outMsg->header.stamp);

	outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
  // fprintf(stderr,"stamp:%d\n",(int)outMsg->header.stamp );
	outMsg->header.frame_id = scanMsg->header.frame_id;
	outMsg->height = 1;
	if(outMsg->width == 0)
		return;
	output_.publish(outMsg);
  //int nn= outMsg->width;
  //fprintf(stderr,"outMsg.width:%d\n",nn);
	wanji_rawdata::VPointCloud cloud;  //typedef wanji_pointcloud::PointXYZIR VPoint;typedef pcl::PointCloud<VPoint> VPointCloud;
	cloud.width    = 5;
 	cloud.height   = 1;
 	cloud.is_dense = false;  //不是稠密型的
 	cloud.points.resize (cloud.width * cloud.height);  //点云总数大小
	cloud = *outMsg.get();
  //pcl::io::savePCDFile ("/home/pl/data/" + std::to_string(a) + ".pcd", cloud);
  }
} // namespace wanji_pointcloud
