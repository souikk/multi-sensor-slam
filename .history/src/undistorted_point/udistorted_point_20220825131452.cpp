/*
 * @Author: Ke Zhang
 * @Date: 2022-08-24 17:46:40
 * @LastEditTime: 2022-08-25 13:13:53
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
        double cloud_start_time = rclcpp::Time(raw_cloud_msg.header.stamp).seconds();

        pcl::PointCloud<PandarPointXYZIRT> raw_cloud;
        pcl::moveFromROSMsg(raw_cloud_msg, raw_cloud);

        double cloud_end_time = raw_cloud.end()->timestamp;
        double diff = cloud_end_time - cloud_start_time;
        while (imu_data.size() >= 2 && imu_data.front().time < cloud_start_time)
        {
            if (imu_data.at(1) > cloud_start_time)
            {
                break;
            }
            imu_data.pop_front();
        }
        if (imu_data.size() < 2)
            return false;

        while (gnss_data.size() >= 2 && gnss_data.front().time < cloud_start_time)
        {
            if (gnss_data.at(1) > cloud_start_time)
            {
                break;
            }
            gnss_data.pop_front();
        }
        if (gnss_data.size() < 2)
            return false;

        double imu_start = imu_data.front().time;
        double imu_angle_x[1000];
        double imu_angle_y[1000];
        double imu_angle_z[1000];
        double imu_time[1000];
        doulbe last_time = imu_start;

        while (imu_data.front().time < cloud_end_time + 0.05)
        {
        }
        for (size_t i = 0; i < raw_cloud.size(); i++)
        {
        }
    }
} // namespace localization
