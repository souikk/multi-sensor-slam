/*
 * @Author: Ke Zhang
 * @Date: 2022-08-24 17:46:40
 * @LastEditTime: 2022-08-25 09:37:32
 * @Description:
 */

#include "undistorted_point/undistorted_point.hpp"

namespace localization
{
    undistortedPoint::undistortedPoint(int lidar)
    {
        lidar_ = lidarType(lidar);
    }

    /**
     * @description: 初始化lidar型号
     * @return {*}
     */
    void undistortedPoint::initLidar()
    {
    }
} // namespace localization
