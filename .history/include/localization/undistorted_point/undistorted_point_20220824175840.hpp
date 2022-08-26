/*
 * @Author: Ke Zhang
 * @Date: 2022-08-24 10:40:25
 * @LastEditTime: 2022-08-24 17:58:40
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
        undistortedPoint();

    public:
        enum lidarType
        {
            Pandar

        };

    private:
        lidarType lidar;
        CloudData undistortedCloud;
    };
} // namespace localization
