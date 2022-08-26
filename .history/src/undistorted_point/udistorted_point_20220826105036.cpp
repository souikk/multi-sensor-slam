/*
 * @Author: Ke Zhang
 * @Date: 2022-08-24 17:46:40
 * @LastEditTime: 2022-08-26 10:49:39
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
                                  Eigen::Affine3d imu2lidar,
                                  Eigen::Affine3d gnss2lidar,
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
            if (imu_data.at(1).time > cloud_start_time)
            {
                break;
            }
            imu_data.pop_front();
        }
        if (imu_data.size() < 2)
            return false;

        while (gnss_data.size() >= 2 && gnss_data.front().time < cloud_start_time)
        {
            if (gnss_data.at(1).time > cloud_start_time)
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
        Eigen::Quaterniond imu_rot[1000];
        double last_imu_time = imu_start;
        Eigen::Matrix3d last_rot = Eigen::Matrix3d::Identity();
        imu_rot[0] = last_rot;
        imu_time[0] = imu_start;
        int num = 0;
        for (size_t i = 0; i < imu_data.size() - 1; i++)
        {
            Eigen::Vector3d gyr1(imu_data[i].angular_velocity.x, imu_data[i].angular_velocity.y, imu_data[i].angular_velocity.z);
            gyr1 = imu2lidar * gyr1;
            Eigen::Vector3d gyr2(imu_data[i + 1].angular_velocity.x, imu_data[i + 1].angular_velocity.y, imu_data[i + 1].angular_velocity.z);
            gyr2 = imu2lidar * gyr2;
            Eigen::Vector3d angle = (imu_data[i + 1].time - imu_data[i].time) * (gyr1 + gyr2) / 2;
            //! 小角度计算旋转矩阵与旋转顺序无关
            Eigen::AngleAxisd roll(angle[0], Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitch(angle[1], Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yaw(angle[2], Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond rot = yaw * pitch * roll;
            imu_rot[i + 1] = last_rot * rot;
            imu_time[i + 1] = imu_data[i + 1].time;

            last_rot = imu_rot[i];
            num++;
            if (imu_data[i + 1].time > cloud_end_time + 0.05)
                break;
        }

        for (size_t i = 0; i < raw_cloud.size(); i++)
        {
            double point_time = raw_cloud[i].timestamp;
            //# 计算这一点到第一个时刻的imu的旋转
            int j = 0;

            for (; j < num; j++)
            {
                if (point_time < imu_time[j])
                {
                    break;
                }
            }
            double ratio = (point_time - imu_time[j - 1]) / (imu_time[j] - imu_time[j - 1]);
            Eigen::Quaterniond cur_point_rot = imu_rot[j - 1].slerp(ratio, imu_rot[j]);

            //# 计算这一点到第一个时刻的gnss的位移
            j = 0;
            for (; j < (int)gnss_data.size(); j++)
            {
                if (point_time < gnss_data[j].time)
                {
                }
            }
        }
    }
} // namespace localization
