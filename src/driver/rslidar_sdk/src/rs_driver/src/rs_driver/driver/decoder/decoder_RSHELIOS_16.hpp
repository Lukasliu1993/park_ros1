/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once
#include <rs_driver/driver/decoder/decoder_RSHELIOS.hpp>
namespace robosense
{
namespace lidar
{

template <typename T_Point>
class DecoderRSHELIOS_16 : public DecoderBase<T_Point>
{
public:
  explicit DecoderRSHELIOS_16(const RSDecoderParam& param, const LidarConstantParameter& lidar_const_param);
  RSDecoderResult decodeDifopPkt(const uint8_t* pkt);
  RSDecoderResult decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height, int& azimuth);
  double getLidarTime(const uint8_t* pkt);
};

template <typename T_Point>
inline DecoderRSHELIOS_16<T_Point>::DecoderRSHELIOS_16(const RSDecoderParam& param,
                                                 const LidarConstantParameter& lidar_const_param)
  : DecoderBase<T_Point>(param, lidar_const_param)
{
  this->vert_angle_list_.resize(this->lidar_const_param_.LASER_NUM);
  this->hori_angle_list_.resize(this->lidar_const_param_.LASER_NUM);
  this->beam_ring_table_.resize(this->lidar_const_param_.LASER_NUM);
  if (this->param_.max_distance > 150.0f)
  {
    this->param_.max_distance = 150.0f;
  }
  if (this->param_.min_distance < 0.1f || this->param_.min_distance > this->param_.max_distance)
  {
    this->param_.min_distance = 0.1f;
  }
}

template <typename T_Point>
inline double DecoderRSHELIOS_16<T_Point>::getLidarTime(const uint8_t* pkt)
{
  return this->template calculateTimeUTC<RSHELIOSMsopPkt>(pkt, LidarType::RSHELIOS);
}

template <typename T_Point>
inline RSDecoderResult DecoderRSHELIOS_16<T_Point>::decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec,
                                                               int& height, int& azimuth)
{
  height = this->lidar_const_param_.LASER_NUM;

  const RSHELIOSMsopPkt* mpkt_ptr = reinterpret_cast<const RSHELIOSMsopPkt*>(pkt);
  if (mpkt_ptr->header.id != this->lidar_const_param_.MSOP_ID)
  {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }

  this->protocol_ver_ = RS_SWAP_SHORT(mpkt_ptr->header.protocol_version);
  this->current_temperature_ = this->computeTemperature(mpkt_ptr->header.temp_raw);
  double block_timestamp = this->get_point_time_func_(pkt);

  azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[0].azimuth);

  this->check_camera_trigger_func_(azimuth, pkt);

  float azi_diff = 0;
  for (size_t blk_idx = 0; blk_idx < this->lidar_const_param_.BLOCKS_PER_PKT; blk_idx++)
  {
    if (mpkt_ptr->blocks[blk_idx].id != this->lidar_const_param_.BLOCK_ID)
    {
      break;
    }

    int cur_azi = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].azimuth);

    if (blk_idx == 0)
    {
      azi_diff = static_cast<float>((RS_ONE_ROUND + RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx + 1].azimuth) - cur_azi) %
                                    RS_ONE_ROUND);
    }
    else
    {
      azi_diff = static_cast<float>((RS_ONE_ROUND + cur_azi - RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx - 1].azimuth)) %
                                    RS_ONE_ROUND);
      block_timestamp = (azi_diff > 100) ? (block_timestamp + this->fov_time_jump_diff_) :
                                           (block_timestamp + this->time_duration_between_blocks_);
    }

    azi_diff = (azi_diff > 100) ? this->azi_diff_between_block_theoretical_ : azi_diff;

    for (int channel_idx = 0; channel_idx < this->lidar_const_param_.CHANNELS_PER_BLOCK; channel_idx++)
    {
      static const float tss[32] = 
      {
         0.00f,  3.15f,  6.30f,  9.45f, 13.26f, 17.08f, 20.56f, 23.71f,
        26.53f, 27.77f, 31.49f, 32.73f, 36.46f, 38.94f, 41.42f, 43.91f,
        55.56f, 58.70f, 61.85f, 65.00f, 68.82f, 72.64f, 76.12f, 79.27f, 
        82.08f, 83.32f, 87.05f, 88.29f, 92.01f, 94.50f, 96.98f, 99.46f
      };

      static const float blk_ts = 55.56f;

      float azi_channel_ori = cur_azi;

      if (this->echo_mode_ == ECHO_SINGLE) // single wave
      {
        azi_channel_ori += azi_diff * tss[channel_idx] / (blk_ts * 2);
      }
      else // dual wave
      {
        azi_channel_ori += azi_diff * tss[channel_idx % 16] / blk_ts;
      }

      int azi_channel_final = this->azimuthCalibration(azi_channel_ori, channel_idx % 16);

      float distance = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].channels[channel_idx].distance) *
                       this->lidar_const_param_.DIS_RESOLUTION;

      int angle_horiz = (int)(azi_channel_ori + RS_ONE_ROUND) % RS_ONE_ROUND;
      int angle_vert = ((this->vert_angle_list_[channel_idx % 16]) + RS_ONE_ROUND) % RS_ONE_ROUND;

      T_Point point;
      if ((distance <= this->param_.max_distance && distance >= this->param_.min_distance) &&
          ((this->angle_flag_ && azi_channel_final >= this->start_angle_ && azi_channel_final <= this->end_angle_) ||
           (!this->angle_flag_ &&
            ((azi_channel_final >= this->start_angle_) || (azi_channel_final <= this->end_angle_)))))
      {
        float x = distance * this->checkCosTable(angle_vert) * this->checkCosTable(azi_channel_final) +
                  this->lidar_const_param_.RX * this->checkCosTable(angle_horiz);
        float y = -distance * this->checkCosTable(angle_vert) * this->checkSinTable(azi_channel_final) -
                  this->lidar_const_param_.RX * this->checkSinTable(angle_horiz);
        float z = distance * this->checkSinTable(angle_vert) + this->lidar_const_param_.RZ;
        uint8_t intensity = mpkt_ptr->blocks[blk_idx].channels[channel_idx].intensity;

        this->transformPoint(x, y, z);

        setX(point, x);
        setY(point, y);
        setZ(point, z);
        setIntensity(point, intensity);
      }
      else
      {
        setX(point, NAN);
        setY(point, NAN);
        setZ(point, NAN);
        setIntensity(point, 0);
      }

      setRing(point, this->beam_ring_table_[channel_idx % 16]);

      if ((this->echo_mode_ == ECHO_SINGLE) && (channel_idx >= 16))
      {
        setTimestamp(point, block_timestamp + this->time_duration_between_blocks_ / 2);
      }
      else
      {
        setTimestamp(point, block_timestamp);
      }

      vec.emplace_back(std::move(point));
    }
  }

  return RSDecoderResult::DECODE_OK;
}

template <typename T_Point>
inline RSDecoderResult DecoderRSHELIOS_16<T_Point>::decodeDifopPkt(const uint8_t* pkt)
{
  RSHELIOSDifopPkt* dpkt_ptr = (RSHELIOSDifopPkt*)pkt;
  if (dpkt_ptr->id != this->lidar_const_param_.DIFOP_ID)
  {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }
  this->template decodeDifopCommon<RSHELIOSDifopPkt>(pkt, LidarType::RSHELIOS);
  if (!this->difop_flag_)
  {
    this->template decodeDifopCalibration<RSHELIOSDifopPkt>(pkt, LidarType::RSHELIOS);
  }

  return RSDecoderResult::DECODE_OK;
}

}  // namespace lidar
}  // namespace robosense
