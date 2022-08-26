/*
 * @Author: Ke Zhang
 * @Date: 2022-08-24 10:40:25
 * @LastEditTime: 2022-08-25 09:44:20
 * @Description:
 */
#pragma once

#include <deque>
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
        bool deskew();
        lidarType lidar_;
        CloudData deskewedCloud;
    };
} // namespace localization
