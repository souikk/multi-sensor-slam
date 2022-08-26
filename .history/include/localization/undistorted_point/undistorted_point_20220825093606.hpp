/*
 * @Author: Ke Zhang
 * @Date: 2022-08-24 10:40:25
 * @LastEditTime: 2022-08-25 09:36:06
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
        undistortedPoint(int lidar);

    private:
        void initLidar();

        lidarType lidar_;
        CloudData deskewedCloud;
    };
} // namespace localization
