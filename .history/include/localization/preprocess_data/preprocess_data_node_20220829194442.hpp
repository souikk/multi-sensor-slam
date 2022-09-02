/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 09:55:26
 * @LastEditTime: 2022-08-29 19:44:41
 * @Description:数据预处理节点头文件
 */
#pragma once

#include <yaml-cpp/yaml.h>

#include <deque>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "glog/logging.h"

#include "sensor_data/imu_data.hpp"
#include "sensor_data/gnss_data.hpp"
#include "sensor_data/cloud_data.hpp"
#include "cloud_type/cloud_type.hpp"
#include "global_path.h"

enum lidarType
{
    Pandar

};

namespace localization
{
    class PreprocessDataNode : public rclcpp::Node
    {
    public:
        using PointXYZIRT = PandarPointXYZIRT;
        PreprocessDataNode(const rclcpp::NodeOptions &options);

        bool preprocessCloud(std::deque<ImuData> &imu_data,
                             std::deque<GnssData> &gnss_data,
                             Eigen::Affine3d imu2lidar,
                             Eigen::Affine3d gnss2lidar,
                             sensor_msgs::msg::PointCloud2 raw_cloud,
                             CloudData deskewedCloud);
        bool deskew(std::deque<ImuData> &imu_data,
                    std::deque<GnssData> &gnss_data,
                    Eigen::Affine3d imu2lidar,
                    Eigen::Affine3d gnss2lidar,
                    sensor_msgs::msg::PointCloud2 raw_cloud,
                    CloudData deskewedCloud);

    private:
        void initLidar();
        bool currentCloudInfo(sensor_msgs::msg::PointCloud2 currentCloudMsg);
        bool imuDeskewInfo(std::deque<ImuData> &imu_data);
        bool gnssDeskewInfo(std::deque<GnssData> &gnss_data);
        double pointDistance(CloudData::pointType point);

        pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn_;
        double lidar_min_range_;
        double lidar_max_range_;
        int downsample_rate_;
        int n_scan_;
        int horizon_scan_;
        lidarType lidar_;
        int number_of_cores_;

        bool deskew_flag_;
        double timeScanCur_;
        double timeScanEnd_;
    };
} // namespace localization