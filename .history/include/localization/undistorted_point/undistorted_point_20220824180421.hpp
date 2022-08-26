/*
 * @Author: Ke Zhang
 * @Date: 2022-08-24 10:40:25
 * @LastEditTime: 2022-08-24 18:04:21
 * @Description:
 */
#pragma once

#include <pcl/point_types.h>

#include "sensor_data/cloud_data.hpp"

namespace localization
{
    class undistortedPoint
    {
    public:
        undistortedPoint(lidarType lidar);

    public:
        enum lidarType
        {
            Pandar

        };

    private:
        lidarType lidar;
        CloudData deskewedCloud;
    };
} // namespace localization
