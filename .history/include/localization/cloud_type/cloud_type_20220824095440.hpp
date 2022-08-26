/*
 * @Author: Ke Zhang
 * @Date: 2022-08-24 09:50:10
 * @LastEditTime: 2022-08-24 09:54:21
 * @Description:
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace localization
{
    struct PandarPointXYZIRT
    {
        PCL_ADD_POINT4D
        uint8_t intensity;
        double timestamp;
        uint16_t ring;                  ///< laser ring number
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
    } EIGEN_ALIGN16;
    // enforce SSE padding for correct memory alignment
    POINT_CLOUD_REGISTER_POINT_STRUCT(PandarPointXYZIRT,
                                      (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(double, timestamp, timestamp)(uint16_t, ring, ring))
} // namespace localization
