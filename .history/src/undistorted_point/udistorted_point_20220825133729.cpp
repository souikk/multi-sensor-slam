/*
 * @Author: Ke Zhang
 * @Date: 2022-08-24 17:46:40
 * @LastEditTime: 2022-08-25 13:37:01
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
        double last_imu_time = imu_start;
        int num = 0;
        for (size_t i = 0; i < imu_data.size(); i++)
        {
            if (i == 0)
            {
                imu_angle_x[i] = 0;
                imu_angle_y[i] = 0;
                imu_angle_z[i] = 0;
                imu_time = imu_data[0].time;
                num++;
                continue;
            }
            imu_angle_x[i] = imu_angle_x[i - 1] + imu_data[i].angular_velocity.x * (imu_data[i].time - last_imu_time);
            imu_angle_y[i] = imu_angle_y[i - 1] + imu_data[i].angular_velocity.y * (imu_data[i].time - last_imu_time);
            imu_angle_z[i] = imu_angle_z[i - 1] + imu_data[i].angular_velocity.z * (imu_data[i].time - last_imu_time);
            imu_time[i] = imu_data[i].time;
            last_imu_time = imu_data[i].time;
            num++;
            if (imu_data[i].time > cloud_end_time)
                break;
        }

        for (size_t i = 0; i < raw_cloud.size(); i++)
        {
            double point_time = raw_cloud[i].timestamp;
            int j = 0;
            for (; j < num; j++)
            {
                if (point_time < imu_time[j])
                {
                    break;
                }
            }
        }
    }
} // namespace localization
