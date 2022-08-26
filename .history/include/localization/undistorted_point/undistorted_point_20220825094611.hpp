/*
 * @Author: Ke Zhang
 * @Date: 2022-08-24 10:40:25
 * @LastEditTime: 2022-08-25 09:46:02
 * @Description:
 */
#pragma once

#include <deque>
#include <pcl/point_types.h>

#include "glog/logging.h"

#include "sensor_data/imu_data.hpp"
#include "sensor_data/gnss_data.hpp"
#include "sensor_data/cloud_data.hpp"

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
        bool deskew();
        lidarType lidar_;
        CloudData deskewedCloud;
    };
} // namespace localization
