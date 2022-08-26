/*
 * @Author: Ke Zhang
 * @Date: 2022-08-24 10:40:25
 * @LastEditTime: 2022-08-25 10:17:52
 * @Description:
 */
#pragma once

#include <deque>
#include <sensor_msgs/msg/point_cloud2.hpp>
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

        bool deskew(std::deque<ImuData> &imu_data,
                    std::deque<GnssData> &gnss_data,
                    sensor_msgs::msg::PointCloud2 raw_cloud,
                    CloudData deskewedCloud);

    private:
        void initLidar();

        lidarType lidar_;
    };
} // namespace localization