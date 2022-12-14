/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011, 2012 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id: data_base.h 1554 2011-06-14 22:11:17Z jack.oquin $
 */

/** \file
 *
 *  Point Cloud Library point structures for Wanji data.
 *
 *  @author Jesse Vera
 *  @author Jack O'Quin
 *  @author Piyush Khandelwal
 */

#ifndef __WANJI_POINTCLOUD_POINT_TYPES_H
#define __WANJI_POINTCLOUD_POINT_TYPES_H

#include <pcl/point_types.h>

namespace wanji_pointcloud
{
  /** Euclidean Wanji coordinate, including intensity and ring number. */
  struct PointXYZIR
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    double timetemp;                    // lidar time
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

}; // namespace wanji_pointcloud


POINT_CLOUD_REGISTER_POINT_STRUCT(wanji_pointcloud::PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (double ,timetemp, timetemp)
                                  )
                                  
#endif // __WANJI_POINTCLOUD_POINT_TYPES_H

