/*
 * @Author: Ke Zhang
 * @Date: 2022-08-24 17:46:40
 * @LastEditTime: 2022-08-24 18:44:29
 * @Description:
 */

#include "undistorted_point/undistorted_point.hpp"

namespace localization
{
    undistortedPoint::undistortedPoint(lidarType lidar)
    {
        LOG(INFO) << "input lidar type :" << lidar;
    }
} // namespace localization
