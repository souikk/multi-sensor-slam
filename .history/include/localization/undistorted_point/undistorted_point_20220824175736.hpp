/*
 * @Author: Ke Zhang
 * @Date: 2022-08-24 10:40:25
 * @LastEditTime: 2022-08-24 17:56:59
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
    };
} // namespace localization
