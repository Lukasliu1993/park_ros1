/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the RILIDAR 3D LIDARs
 */
#include "wanji_driver/wanji_driver.h"
#include <wanji_msgs/WanjiScan.h>

namespace wanji_driver
{
  static const unsigned int POINTS_ONE_CHANNEL_PER_SECOND = 20000;
  static const unsigned int BLOCKS_ONE_CHANNEL_PER_PKT = 12;

wanjiDriver::wanjiDriver(ros::NodeHandle node, ros::NodeHandle private_nh)
{
  // use private node handle to get parameters
  private_nh.param("frame_id", config_.frame_id, std::string("wanji"));

  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);

  // get model name, validate string, determine packet rate
  private_nh.param("model", config_.model, std::string("16E"));
  double packet_rate;  // packet frequency (Hz)

  packet_rate = 1800;   //20000/24

  private_nh.param("rpm", config_.rpm, 600);
  fprintf(stderr,"driver_rpm:%d\n",config_.rpm);
  double frequency = (config_.rpm / 60.0);  // expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)

  int npackets = (int)ceil(packet_rate / frequency);
  private_nh.param("npackets", config_.npackets, npackets);
  ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

  int msop_udp_port;
  std::string dump_file;
  private_nh.param("pcap", dump_file, std::string(""));

  private_nh.param("port", msop_udp_port, 3001);
  // int difop_udp_port;
  // private_nh.param("difop_port", difop_udp_port, (int)DIFOP_DATA_PORT_NUMBER);
	srv_ = boost::make_shared<
			dynamic_reconfigure::Server<wanji_driver::WanjiNodeConfig> >(
			private_nh);
	dynamic_reconfigure::Server<wanji_driver::WanjiNodeConfig>::CallbackType f;
	f = boost::bind(&wanjiDriver::callback, this, _1, _2);
	srv_->setCallback(f); // Set callback function und call initially

  // open rslidar input device or file
  if (dump_file != "")  // have PCAP file?
  {
    // read data from packet capture file
    msop_input_.reset(new wanji_driver::InputPCAP(private_nh, msop_udp_port, packet_rate, dump_file));
  }
  else
  {
    // read data from live socket
    msop_input_.reset(new wanji_driver::InputSocket(private_nh, msop_udp_port));
  }

  // raw packet output topic
  std::string output_packets_topic;
  private_nh.param("output_packets_topic", output_packets_topic, std::string("wanji_packets"));
  msop_output_ = node.advertise<wanji_msgs::WanjiScan>(output_packets_topic, 10);

}


wanjiDriver::~wanjiDriver()
{
  if (difop_thread_ !=NULL)
  {
    difop_thread_->interrupt();
    difop_thread_->join();
  }
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool wanjiDriver::poll(void)
{  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
    static int hadDropPacketFlag=0;
    wanji_msgs::WanjiScanPtr scan(new wanji_msgs::WanjiScan);
    scan->packets.resize(config_.npackets);
    static int firstpro = 0;
    msop_input_->resleepPackets(config_.rpm);

    for (int i = 0; i < config_.npackets; ++i)
    {
      while (true)
      {
        // keep reading until full packet received
        int rc = msop_input_->getPacket(&scan->packets[i], config_.time_offset);
        if (rc == 0)
        {
          break;  // got a full packet?
        }
          
        if (rc < 0)
          return false;  // end of file reached?
      }
      
      unsigned char *pdata = static_cast<unsigned char *>(&scan->packets[i].data[0]);
      if(pdata[0]==0xFF && pdata[1]==0xDD)
      {
        int pcakang = config_.rpm/30*pdata[4];
        if((((pdata[5] | (pdata[6]<<8)) - firstpro) != pcakang) && (((pdata[5] | (pdata[6]<<8)) - firstpro) != (pcakang-36000)))
        {
          //fprintf(stderr,"nowazimuth %d lastazimuth:%d ID%d\n",pdata[5] | (pdata[6]<<8),firstpro,pdata[1253-57] | (pdata[1253-56]<<8));
          firstpro = (pdata[5] | (pdata[6]<<8));
          return true;
        }
        firstpro = (pdata[5] | (pdata[6]<<8));
      }
      
      //publish packet from horizon angle 0
      if(hadDropPacketFlag == 0)
      {
        int first_azimuth;
        if(pdata[0]==0xFF && pdata[1]==0xEE)//header:0xFFEE
        {
          first_azimuth = pdata[2] | (pdata[3]<<8) ;
        }
        else if(pdata[0]==0xFF && pdata[1]==0xDD)//header:0xFFDD
        {
          first_azimuth = pdata[5] | (pdata[6]<<8);          
        }
        else
        {
          return false;
        }

        first_azimuth  = first_azimuth%36000;
        
        if(first_azimuth  == 0)
        {
          hadDropPacketFlag = 1;
          msop_input_->refreshPackets(config_.npackets,pdata,config_.rpm);
          fprintf(stderr,"0 degree will publish from next packet\n");
          return true;
        }
      }      
    }
  // publish message using time of last packet read
  //ROS_INFO("Publishing a full scan.");
  scan->header.stamp = scan->packets.back().stamp;
  scan->header.frame_id = config_.frame_id;
  msop_output_.publish(scan);
  return true;
}

void wanjiDriver::callback(wanji_driver::WanjiNodeConfig &config,
		uint32_t level) {
	ROS_INFO("Reconfigure Request,time_offset=%f",config.time_offset);
	config_.time_offset = config.time_offset;
}
// add for time synchronization
}  // namespace wanji_driver
