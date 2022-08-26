/*
 * @Author: Ke Zhang
 * @Date: 2022-08-24 17:46:40
 * @LastEditTime: 2022-08-25 10:53:33
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

    bool undistortedPoint::deskew(std::deque<ImuData> &imu_data,
                                  std::deque<GnssData> &gnss_data,
                                  sensor_msgs::msg::PointCloud2 raw_cloud_msg,
                                  CloudData deskewedCloud)
    {
        double cloud_time = rclcpp::Time(raw_cloud_msg.header.stamp).seconds();
        pcl::PointCloud<PandarPointXYZIRT> raw_cloud;
        pcl::moveFromROSMsg(raw_cloud_msg, raw_cloud);
    }
} // namespace localization
