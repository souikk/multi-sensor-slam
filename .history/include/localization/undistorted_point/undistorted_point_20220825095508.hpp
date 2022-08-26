/*
 * @Author: Ke Zhang
 * @Date: 2022-08-24 10:40:25
 * @LastEditTime: 2022-08-25 09:54:51
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
        bool deskew(CloudData raw_cloud, std::deque<ImuData> &imu_data, std::deque<GnssData> &gnss_data);

    private:
        void initLidar();

        lidarType lidar_;
        CloudData deskewedCloud;
    };
} // namespace localization
