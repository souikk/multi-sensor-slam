/*
 * @Author: Ke Zhang
 * @Date: 2022-08-24 17:46:40
 * @LastEditTime: 2022-08-25 09:55:26
 * @Description:
 */

#include "undistorted_point/undistorted_point.hpp"

namespace localization
{
    undistortedPoint::undistortedPoint(int lidar)
    {
        lidar_ = lidarType(lidar);
        initLidar();
    }

    /**
     * @description: 初始化lidar型号
     * @return {*}
     */
    void undistortedPoint::initLidar()
    {
        if (lidar_ == lidarType::Pandar)
        {
            using PointXYZIRT = PandarPointXYZIRT;
        }
        else
        {
            LOG(ERROR) << "unkown lidar type.";
        }
    }

    bool undistortedPoint::deskew(CloudData raw_cloud, std::deque<ImuData> &imu_data, std::deque<GnssData> &gnss_data)
    {
    }
} // namespace localization
