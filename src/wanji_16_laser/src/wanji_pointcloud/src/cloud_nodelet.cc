/*
 * This file is part of lslidar_n301 driver.
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

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "wanji_pointcloud/convert.h"

namespace wanji_pointcloud
{
class CloudNodelet : public nodelet::Nodelet
{
public:
  CloudNodelet()
  {
  }
  ~CloudNodelet()
  {
  }

private:
  virtual void onInit();
  boost::shared_ptr<Convert> conv_;
};

/** @brief Nodelet initialization. */
void CloudNodelet::onInit()
{
  conv_.reset(new Convert(getNodeHandle(), getPrivateNodeHandle()));
}

}  // namespace wanji_pointcloud

// parameters: class type, base class type
PLUGINLIB_EXPORT_CLASS(wanji_pointcloud::CloudNodelet, nodelet::Nodelet)
