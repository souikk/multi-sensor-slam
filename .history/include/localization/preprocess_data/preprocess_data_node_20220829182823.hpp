/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 09:55:26
 * @LastEditTime: 2022-08-29 18:28:23
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

using PointXYZIRT = PandarPointXYZIRT;

enum lidarType
{
    Pandar

};

namespace localization
{
    class PreprocessDataNode : public rclcpp::Node
    {
    public:
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
        double pointDistance(CloudData::pointType point);

        double lidarMinRange_;
        double lidarMaxRange_;
        int downsampleRate_;
        int N_SCAN_;
        int Horizon_SCAN_;
        lidarType lidar_;

        bool deskew_flag_;
        double timeScanCur_;
        double timeScanEnd_;
    };
} // namespace localization