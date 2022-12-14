/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  Wanji 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw Wanji LIDAR packets into useful
 *  formats.
 *
 *  Derived classes accept raw Wanji data for either single packets
 *  or entire rotations, and provide it in various formats for either
 *  on-line or off-line processing.
 *
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 *
 *  HDL-64E S2 calibration support provided by Nick Hillier
 */

#include <fstream>
#include <sstream>
 #include <iostream>
#include <stdio.h>
#include <math.h>
#include <string>
#include <ctime>
#include <chrono>
using namespace std;

#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>
#include <boost/thread/lock_guard.hpp>

#include <pcl/common/transforms.h>
#include <wanji_pointcloud/rawdata.h>
#define HDL_Grabber_toRadians(x) ((x)*M_PI/180.0)
namespace wanji_rawdata
{
  ////////////////////////////////////////////////////////////////////////
  //
  // RawData base class implementation
  //
  ////////////////////////////////////////////////////////////////////////

  RawData::RawData():
  line_mask_(0)
  {
	//intensity_vector_.resize(360*8,1);
	//this->point_cloud_all_.reset(new VPointCloud);

	lidar_pos_vec_ << 0,0,0;
	lidar_atti_vec_ << 0,0,0;
	this->transform_.setIdentity();
	this->transform_.translate(this->lidar_pos_vec_);
	Eigen::Matrix3f rotation;
	rotation = Eigen::AngleAxisf(this->lidar_atti_vec_[0]/180*M_PI, Eigen::Vector3f::UnitX())
			  * Eigen::AngleAxisf(this->lidar_atti_vec_[1]/180*M_PI, Eigen::Vector3f::UnitY())
			  * Eigen::AngleAxisf(this->lidar_atti_vec_[2]/180*M_PI, Eigen::Vector3f::UnitZ());
	angleResolutionVal=0;
	lineAngleGap=0;
	this->transform_.rotate(rotation);

   }
  

  /** Set up for on-line operation. */
  int RawData::setup(ros::NodeHandle private_nh)
  {
    // Set up cached values for sin and cos of all the possible headings
    for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
      float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
      cos_rot_table_[rot_index] = cosf(rotation);
      sin_rot_table_[rot_index] = sinf(rotation);
    }
    std::string anglePath;
	private_nh.getParam("calibration",anglePath);
	// fprintf(stderr,"anglePath:%s\n",anglePath.c_str());
	
    private_nh.getParam("max_angle",config_.max_angle);
	// fprintf(stderr,"max_angle:%f\n",config_.max_angle);
	private_nh.getParam("min_angle",config_.min_angle);
	// fprintf(stderr,"min_angle:%f\n",config_.min_angle);

	private_nh.getParam("max_distance",config_.max_distance);
	// fprintf(stderr,"max_distance:%f\n",config_.max_distance);
	private_nh.getParam("min_distance",config_.min_distance);
	// fprintf(stderr,"min_distance:%f\n",config_.min_distance);

	private_nh.getParam("rpm",config_.rpm);
	// fprintf(stderr,"rpm:%d\n",config_.rpm);
	switch(config_.rpm)
	{
		case 300:
		   angleResolutionVal=0.1;
           lineAngleGap=0.0025;
		   break;
		case 600:
		   angleResolutionVal=0.2;
		   lineAngleGap=0.005;
		   break;
		case 900:
		   angleResolutionVal=0.3;
		   lineAngleGap=0.0075;
		   break;
		case 1200:
		   angleResolutionVal=0.4;
		   lineAngleGap=0.01;
		   break;
		default:
		   break;

	}
	fprintf(stderr,"angleResolutionVal:%f\n",angleResolutionVal);
	groupAngleVal=angleResolutionVal/4;

 	private_nh.getParam("return_mode",return_mode_);//get return mode.
	setCalibration(private_nh);


	private_nh.param("time_mode",time_mode_,false);
	
	return 0;
  }


  /** Set up for offline operation */
  int RawData::setupOffline(std::string calibration_file, double max_range_, double min_range_)
  {

    //   config_.max_range = max_range_;
    //   config_.min_range = min_range_;
    //   ROS_INFO_STREAM("data ranges to publish: ["
	//       << config_.min_range << ", "
	//       << config_.max_range << "]");

    //   config_.calibrationFile = calibration_file;

    //   ROS_INFO_STREAM("correction angles: " << config_.calibrationFile);

    //   calibration_.read(config_.calibrationFile);
    //   if (!calibration_.initialized) 
	//   {
	//   ROS_ERROR_STREAM("Unable to open calibration file: " <<
	// 	  config_.calibrationFile);
	//   return -1;
    //   }

    //   // Set up cached values for sin and cos of all the possible headings
    //   for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) 
	//   {
	//   float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
	//   cos_rot_table_[rot_index] = cosf(rotation);
	//   sin_rot_table_[rot_index] = sinf(rotation);
    //   }
      return 0;
  }

  /** @brief get average detal Angle of Anzimuth in Dual return mode,total 15(BLOCKS_PER_PACKET) blocks
   *
   *  @param raw raw packet of data
   *  @return return the detal val
   */
	double RawData::getDetalAngleAzi(const raw_packet_t *raw)
	{
		double detalSum=0;
		for(int i=0;i<(BLOCKS_PER_PACKET-1);i++)
		{
			detalSum += (raw->blocks[i+1].rotation + 36000 - raw->blocks[i].rotation) % 36000 / 2;
		}

		return detalSum/(BLOCKS_PER_PACKET-1);
	}

   /** @brief reset calibration. */
	int RawData::setCalibration(ros::NodeHandle private_nh)
	{
		std::string anglePath;
		private_nh.getParam("calibration",anglePath);

		ifstream file;
		file.open(anglePath.c_str());
		unsigned int i=0;
		fprintf(stderr,"set Calibration in pointcloud\n");
		if (!file)
		{
			ROS_WARN_STREAM("[cloud][rawdata] " << anglePath << " does not exist");
			fprintf(stderr,"%s\n",anglePath.c_str());
			return -1;
		}
		else
		{
			string wow, mem, key;
			unsigned int x = 0;
			while(true)
			{
				getline(file, wow);
				if (file.fail()) break; //check for error
				while (x < wow.length() ) 
				{
				if (wow[x] == ',')
				{
					key = mem;
					mem.clear();
					x++; //step over ','
				} 
				else 
					mem += wow[x++];
				}
				istringstream isAng(key);
				isAng>>VERT_ANGLE[i];
				istringstream isAzimuth(mem);
				isAzimuth>>AzimuthDiff[i];
				// fprintf(stderr,"output:%f,%f\n",VERT_ANGLE[i],AzimuthDiff[i]);
				i++;
					mem.clear(); //reset memory
				key.clear();
					x = 0;//reset index
			}
			file.close();
	
			return 0;
		}
	}

  /** @brief convert raw packet to point cloud
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
 void RawData::unpack(const wanji_msgs::WanjiPacket &pkt,
                       VPointCloud &pc)
  {
    ROS_DEBUG_STREAM("Received packet, time: " << pkt.stamp);
    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];
	const uint8_t *pdata = (const uint8_t *)&pkt.data[0];
	// fprintf(stderr," packet %02X %02X\n",pkt.data[0],pkt.data[1]);
	if(pdata[0]==0xFF && pdata[1]==0xEE)//0xFFEE 1260B
	{
		double timetemp = setTimetemp((u_int8_t *)&pdata[1200],time_mode_);
		for (int i = 0; i < BLOCKS_PER_PACKET; i++) 
		{
			for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE)
			{
				static int recAzimuth=-1;
				float xy,x, y, z;
				float intensity;
				uint8_t laser_number;       ///< hardware laser number
					laser_number = j;
				union two_bytes tmp;
				tmp.bytes[0] = raw->blocks[i].data[k];
				tmp.bytes[1] = raw->blocks[i].data[k+1];

				float distance = tmp.uint * DISTANCE_RESOLUTION;
				int azimuth=raw->blocks[i].rotation/10-1;
				intensity = raw->blocks[i].data[k+2];
				float confidence = raw->blocks[i].data[k+3];
				int line;

				float azimuthInRadians=0,azimuthAngle=0;
				azimuthAngle=(raw->blocks[i].rotation)/ 100.0;
				if(azimuthAngle>=config_.min_angle&&azimuthAngle<=config_.max_angle)
				{
					static float first_Angazimuth=0,second_Angazimuth=0,third_Angazimuth=0,fourth_Angazimuth=0;
					if(recAzimuth!=azimuth)
					{
						first_Angazimuth=(raw->blocks[i].rotation)/ 100.0-angleResolutionVal;
						second_Angazimuth=first_Angazimuth+groupAngleVal;
						third_Angazimuth=first_Angazimuth+2*groupAngleVal;
						fourth_Angazimuth=first_Angazimuth+3*groupAngleVal;
					}
					switch(j)
					{
						case 0:
						azimuthInRadians=HDL_Grabber_toRadians(first_Angazimuth+AzimuthDiff[j]);
						line = 8;
						break;
						case 1:
						azimuthInRadians=HDL_Grabber_toRadians(first_Angazimuth+lineAngleGap+AzimuthDiff[j]);
						line = 1;
						break;
						case 2:
						azimuthInRadians=HDL_Grabber_toRadians(first_Angazimuth+2*lineAngleGap+AzimuthDiff[j]);
						line = 2;
						break;
						case 3:
						azimuthInRadians=HDL_Grabber_toRadians(first_Angazimuth+3*lineAngleGap+AzimuthDiff[j]);
						line = 3;
						break;
						case 4:
						azimuthInRadians=HDL_Grabber_toRadians(first_Angazimuth+4*lineAngleGap+AzimuthDiff[j]);
						line = 4;
						break;
						case 5:
						azimuthInRadians=HDL_Grabber_toRadians(second_Angazimuth+AzimuthDiff[j]);
						line = 8;
						break;
						case 6:
						azimuthInRadians=HDL_Grabber_toRadians(second_Angazimuth+lineAngleGap+AzimuthDiff[j]);
						line = 5;
						break;
						case 7:
						azimuthInRadians=HDL_Grabber_toRadians(second_Angazimuth+2*lineAngleGap+AzimuthDiff[j]);
						line = 6;
						break;
						case 8:
						azimuthInRadians=HDL_Grabber_toRadians(second_Angazimuth+3*lineAngleGap+AzimuthDiff[j]);
						line = 7;
						break;
						case 9:
						azimuthInRadians=HDL_Grabber_toRadians(third_Angazimuth+AzimuthDiff[j]);
						line = 8;
						break;
						case 10:
						azimuthInRadians=HDL_Grabber_toRadians(third_Angazimuth+lineAngleGap+AzimuthDiff[j]);
						line = 9;
						break;
						case 11:
						azimuthInRadians=HDL_Grabber_toRadians(third_Angazimuth+2*lineAngleGap+AzimuthDiff[j]);
						line = 10;
						break;
						case 12:
						azimuthInRadians=HDL_Grabber_toRadians(third_Angazimuth+3*lineAngleGap+AzimuthDiff[j]);
						line = 11;
						break;
						case 13:
						azimuthInRadians=HDL_Grabber_toRadians(third_Angazimuth+4*lineAngleGap+AzimuthDiff[j]);
						line = 12;
						break;
						case 14:
						azimuthInRadians=HDL_Grabber_toRadians(fourth_Angazimuth+AzimuthDiff[j]);
						line = 8;
						break;
						case 15:
						azimuthInRadians=HDL_Grabber_toRadians(fourth_Angazimuth+lineAngleGap+AzimuthDiff[j]);
						line = 13;
						break;
						case 16:
						azimuthInRadians=HDL_Grabber_toRadians(fourth_Angazimuth+2*lineAngleGap+AzimuthDiff[j]);
						line = 14;
						break;
						case 17:
						azimuthInRadians=HDL_Grabber_toRadians(fourth_Angazimuth+3*lineAngleGap+AzimuthDiff[j]);
						line = 15;
						break;
						case 18:
						azimuthInRadians=HDL_Grabber_toRadians(fourth_Angazimuth+4*lineAngleGap+AzimuthDiff[j]);
						line = 16;
						break;
						default:
						break;
					}
					recAzimuth=azimuth;
					float verticalVal=HDL_Grabber_toRadians(VERT_ANGLE[j]);
					xy=distance*cos(verticalVal);
					x = xy*(sin(azimuthInRadians));
				    y = xy*(cos(azimuthInRadians));
					z = distance*sin(verticalVal);
					VPoint point;
					point.x = x;//_coord;
					point.y = y;//_coord;
					point.z = z;//_coord;
					point.intensity = confidence;
					point.ring = line;
					point.timetemp = timetemp;
// fprintf(stderr,"[%.0f %.0f] ",intensity, confidence);

					// append this point to the cloud
					pc.points.push_back(point);
					++pc.width;
				}
			}
    	}
	}
	else if(pdata[0]==0xFF && pdata[1]==0xDD)//0xFFDD 1001 1352
	{
		int start_idx = 5;
		int channel_nums = pdata[2];
		int echo_nums = pdata[3];
		int block_nums = pdata[4];
		int all_block_nums = block_nums*echo_nums;
		int block_size = 2+4*channel_nums;
		int channel_size = 4;
		int len_data = (channel_nums * 4 + 2)*all_block_nums+65;
		double timetemp = setTimetemp((u_int8_t *)&pdata[len_data-60],time_mode_);
		// fprintf(stderr,"channel_nums:%d block_nums:%d all_block_nums:%d\n",channel_nums,block_nums,all_block_nums);

		if(channel_nums == 16)
		{
			for (int i = 0; i < all_block_nums; i++)
			{
				const uint8_t *block_ptr = &(pdata[start_idx + i * block_size]);
				union two_bytes tmp;
				tmp.bytes[0] = block_ptr[0];
				tmp.bytes[1] = block_ptr[1];
				int azimuth = tmp.uint;
				//fprintf(stderr,"azimuth:%d\n",azimuth);
				for (int j = 0, k = 0; j < channel_nums; j++, k += channel_size)
				{
					static int recAzimuth = -1;
					float xy, x, y, z;
					float intensity;
					float confidence;
					int azimuth_id = tmp.uint / 10 - 1;

					const uint8_t *channel_ptr = &(pdata[start_idx + i * block_size + 2 + j * channel_size]);
					tmp.bytes[0] = channel_ptr[0];
					tmp.bytes[1] = channel_ptr[1];
					float distance = tmp.uint * DISTANCE_RESOLUTION;
					intensity = channel_ptr[2];
					confidence = channel_ptr[3];
					// fprintf(stderr,"distance:%.1f\n",distance);
					int line;
					float verticalVal = 0;
					float azimuthInRadians = 0, azimuthAngle = 0;
					azimuthAngle = (azimuth) / 100.0;
					if (azimuthAngle >= config_.min_angle && azimuthAngle <= config_.max_angle)
					{
						azimuthInRadians = HDL_Grabber_toRadians(azimuthAngle);
						line = j + 1;
						switch (j)
						{
						case 0:
							verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[1]);
							break;
						case 1:
							verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[2]);
							break;
						case 2:
							verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[3]);
							break;
						case 3:
							verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[4]);
							break;
						case 4:
							verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[6]);
							break;
						case 5:
							verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[7]);
							break;
						case 6:
							verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[8]);
							break;
						case 7:
							verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[9]);
							break;
						case 8:
							verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[10]);
							break;
						case 9:
							verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[11]);
							break;
						case 10:
							verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[12]);
							break;
						case 11:
							verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[13]);
							break;
						case 12:
							verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[15]);
							break;
						case 13:
							verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[16]);
							break;
						case 14:
							verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[17]);
							break;
						case 15:
							verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[18]);
							break;
						default:
							break;
						}
						recAzimuth = azimuth;

						xy = distance * cos(verticalVal);
						x = xy*(sin(azimuthInRadians));
					    y = xy*(cos(azimuthInRadians));
						z = distance * sin(verticalVal);
						VPoint point;
						point.x = x; //_coord;
						point.y = y; //_coord;
						point.z = z; //_coord;
						point.intensity = confidence;
						point.ring = j + 1;
						point.timetemp = timetemp;
						// append this point to the cloud
						pc.points.push_back(point);
						++pc.width;
					}
				}
			}
		}
		else if(channel_nums == 19)
		{
			for (int i = 0; i < all_block_nums; i++)
			{
				const uint8_t *block_ptr = &(pdata[start_idx + i * block_size]);
				union two_bytes tmp;
				tmp.bytes[0] = block_ptr[0];
				tmp.bytes[1] = block_ptr[1];
				int azimuth = tmp.uint;
				// fprintf(stderr,"azimuth:%d\n",azimuth);
				for (int j = 0, k = 0; j < channel_nums; j++, k += channel_size)
				{
					static int recAzimuth = -1;
					float xy, x, y, z;
					float intensity;
					float confidence;
					int azimuth_id = tmp.uint / 10 - 1;

					const uint8_t *channel_ptr = &(pdata[start_idx + i * block_size + 2 + j * channel_size]);
					tmp.bytes[0] = channel_ptr[0];
					tmp.bytes[1] = channel_ptr[1];
					float distance = tmp.uint * DISTANCE_RESOLUTION;
					intensity = channel_ptr[2];
					confidence = channel_ptr[3];
					// fprintf(stderr,"distance:%.1f\n",distance);
					int line;

					float azimuthInRadians = 0, azimuthAngle = 0;
					azimuthAngle = (azimuth) / 100.0;
					if (azimuthAngle >= config_.min_angle && azimuthAngle <= config_.max_angle)
					{

						static float first_Angazimuth = 0, second_Angazimuth = 0, third_Angazimuth = 0, fourth_Angazimuth = 0;
						if (recAzimuth != azimuth)
						{
							first_Angazimuth = (azimuth) / 100.0 - angleResolutionVal;
							second_Angazimuth = first_Angazimuth + groupAngleVal;
							third_Angazimuth = first_Angazimuth + 2 * groupAngleVal;
							fourth_Angazimuth = first_Angazimuth + 3 * groupAngleVal;
						}
						switch (j)
						{
						case 0:
							azimuthInRadians = HDL_Grabber_toRadians(first_Angazimuth + AzimuthDiff[j]);
							line = 8;
							break;
						case 1:
							azimuthInRadians = HDL_Grabber_toRadians(first_Angazimuth + lineAngleGap + AzimuthDiff[j]);
							line = 1;
							break;
						case 2:
							azimuthInRadians = HDL_Grabber_toRadians(first_Angazimuth + 2 * lineAngleGap + AzimuthDiff[j]);
							line = 2;
							break;
						case 3:
							azimuthInRadians = HDL_Grabber_toRadians(first_Angazimuth + 3 * lineAngleGap + AzimuthDiff[j]);
							line = 3;
							break;
						case 4:
							azimuthInRadians = HDL_Grabber_toRadians(first_Angazimuth + 4 * lineAngleGap + AzimuthDiff[j]);
							line = 5;
							break;
						case 5:
							azimuthInRadians = HDL_Grabber_toRadians(second_Angazimuth + AzimuthDiff[j]);
							line = 8;
							break;
						case 6:
							azimuthInRadians = HDL_Grabber_toRadians(second_Angazimuth + lineAngleGap + AzimuthDiff[j]);
							line = 5;
							break;
						case 7:
							azimuthInRadians = HDL_Grabber_toRadians(second_Angazimuth + 2 * lineAngleGap + AzimuthDiff[j]);
							line = 6;
							break;
						case 8:
							azimuthInRadians = HDL_Grabber_toRadians(second_Angazimuth + 3 * lineAngleGap + AzimuthDiff[j]);
							line = 7;
							break;
						case 9:
							azimuthInRadians = HDL_Grabber_toRadians(third_Angazimuth + AzimuthDiff[j]);
							line = 8;
							break;
						case 10:
							azimuthInRadians = HDL_Grabber_toRadians(third_Angazimuth + lineAngleGap + AzimuthDiff[j]);
							line = 9;
							break;
						case 11:
							azimuthInRadians = HDL_Grabber_toRadians(third_Angazimuth + 2 * lineAngleGap + AzimuthDiff[j]);
							line = 10;
							break;
						case 12:
							azimuthInRadians = HDL_Grabber_toRadians(third_Angazimuth + 3 * lineAngleGap + AzimuthDiff[j]);
							line = 11;
							break;
						case 13:
							azimuthInRadians = HDL_Grabber_toRadians(third_Angazimuth + 4 * lineAngleGap + AzimuthDiff[j]);
							line = 12;
							break;
						case 14:
							azimuthInRadians = HDL_Grabber_toRadians(fourth_Angazimuth + AzimuthDiff[j]);
							line = 13;
							break;
						case 15:
							azimuthInRadians = HDL_Grabber_toRadians(fourth_Angazimuth + lineAngleGap + AzimuthDiff[j]);
							line = 8;
							break;
						case 16:
							azimuthInRadians = HDL_Grabber_toRadians(fourth_Angazimuth + 2 * lineAngleGap + AzimuthDiff[j]);
							line = 14;
							break;
						case 17:
							azimuthInRadians = HDL_Grabber_toRadians(fourth_Angazimuth + 3 * lineAngleGap + AzimuthDiff[j]);
							line = 15;
							break;
						case 18:
							azimuthInRadians = HDL_Grabber_toRadians(fourth_Angazimuth + 4 * lineAngleGap + AzimuthDiff[j]);
							line = 16;
							break;
						default:
							break;
						}
						recAzimuth = azimuth;
						float verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[j]);
						xy = distance * cos(verticalVal);
						x = xy*(sin(azimuthInRadians));
					    y = xy*(cos(azimuthInRadians));
						z = distance * sin(verticalVal);
						VPoint point;
						point.x = x; //_coord;
						point.y = y; //_coord;
						point.z = z; //_coord;
						point.intensity = confidence;
						point.ring = line;
						point.timetemp = timetemp;
						// append this point to the cloud
						pc.points.push_back(point);
						++pc.width;
					}
				}
			}
		}
	}
  }

  void RawData::updatePos(Eigen::Vector3f posvec,Eigen::Vector3f attivec)
  {
  	boost::lock_guard<boost::mutex> lock(this->pos_mutex_);
  	if(posvec == this->lidar_pos_vec_ && attivec == this->lidar_atti_vec_)
  	{
  		return;
  	}
  	else
  	{
  		this->lidar_pos_vec_ = posvec;
  		this->lidar_atti_vec_ = attivec;
  		this->transform_.setIdentity();
  		this->transform_.translate(this->lidar_pos_vec_);
  		Eigen::Matrix3f rotation;
  		rotation = Eigen::AngleAxisf(this->lidar_atti_vec_[0]/180*M_PI, Eigen::Vector3f::UnitX())
  				  * Eigen::AngleAxisf(this->lidar_atti_vec_[1]/180*M_PI, Eigen::Vector3f::UnitY())
  				  * Eigen::AngleAxisf(this->lidar_atti_vec_[2]/180*M_PI, Eigen::Vector3f::UnitZ());
  		this->transform_.rotate(rotation);
  	}
  }

  double RawData::setTimetemp(uint8_t *pdata,bool timetype)
  {
	  if (!timetype)
	  {
		  const auto t = std::chrono::system_clock::now();
          const auto t_sec = std::chrono::duration_cast<std::chrono::duration<double>>(t.time_since_epoch());
    	  return (double)t_sec.count();
	  }
	  else
	  {
		   std::tm stm;
	       memset(&stm, 0, sizeof(stm));
		   
	       stm.tm_year = pdata[9]+100;
	       stm.tm_mon = pdata[8];
	       stm.tm_mday = pdata[7];
	       stm.tm_hour = pdata[6];
	       stm.tm_min = pdata[5];
	       stm.tm_sec = pdata[4];

	       return std::mktime(&stm);
	  }
  }

} // namespace wanji_rawdata
