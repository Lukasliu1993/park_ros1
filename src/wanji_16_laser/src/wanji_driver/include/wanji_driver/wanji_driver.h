/*
 * This file is part of lslidar_c16 driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _LS_C16_DRIVER_H_
#define _LS_C16_DRIVER_H_

#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "wanji_driver/input.h"
#include <dynamic_reconfigure/server.h>
#include <wanji_driver/WanjiNodeConfig.h>
namespace wanji_driver
{
class wanjiDriver
{
public:
  /**
 * @brief wanjiDriver
 * @param node          raw packet output topic
 * @param private_nh    通过这个节点传参数
 */
  wanjiDriver(ros::NodeHandle node, ros::NodeHandle private_nh);

  ~wanjiDriver();

  bool poll(void);

private:
  	///Callback for dynamic reconfigure
	void callback(wanji_driver::WanjiNodeConfig &config, uint32_t level);
  ///Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<wanji_driver::WanjiNodeConfig> > srv_;
  // configuration parameters
  struct
  {
    std::string frame_id;  ///< tf frame ID
    std::string model;     ///< device model name
    int npackets;          ///< number of packets to collect
    int rpm;            ///< device rotation rate (RPMs)
    double time_offset;    ///< time in seconds added to each  time stamp
    int cut_angle;
  } config_;

  boost::shared_ptr<Input> msop_input_;
  boost::shared_ptr<Input> difop_input_;
  ros::Publisher msop_output_;
  ros::Publisher difop_output_;
  ros::Publisher output_sync_;
  // Converter convtor_
  boost::shared_ptr<boost::thread> difop_thread_;

  // add for time synchronization
  bool time_synchronization_;
    unsigned char packetTimeStamp[10];
    uint64_t pointcloudTimeStamp;
    uint64_t GPSStableTS;
    uint64_t GPSCountingTS;
    uint64_t last_FPGA_ts;
    uint64_t GPS_ts;
    struct tm cur_time;
    int cnt_gps_ts;
    ros::Time timeStamp;

};

}  // namespace lslidar_driver

#endif
