/*
 * @Author: Ke Zhang
 * @Date: 2022-08-24 10:40:25
 * @LastEditTime: 2022-08-24 18:26:59
 * @Description:
 */
#pragma once

#include <pcl/point_types.h>

#include "sensor_data/cloud_data.hpp"

#include "glog/logging.h"
enum lidarType
{
    Pandar

};

namespace localization
{
    class undistortedPoint
    {
    public:
        undistortedPoint(lidarType lidar);

    private:
        lidarType lidar;
        CloudData deskewedCloud;
    };
} // namespace localization
