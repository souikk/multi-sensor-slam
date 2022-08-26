/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 09:58:44
 * @LastEditTime: 2022-08-24 18:05:51
 * @Description:数据预处理工作流程类的头文件
 */
#pragma once

#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"

#include "preprocess_data/preprocess_data_node.hpp"
#include "subscriber/lidar_subscriber.hpp"
#include "subscriber/imu_subscriber.hpp"
#include "subscriber/gnss_subscriber.hpp"
#include "undistorted_point/undistorted_point.hpp"
#include "global_path.h"
namespace localization
{
    class PreprocessDataFlow
    {
    public:
        PreprocessDataFlow(rclcpp::Node::SharedPtr node_ptr);

        bool run();

    private:
        void initSubscriber(rclcpp::Node::SharedPtr node_ptr, const YAML::Node &config);
        bool read();
        bool hasData();
        bool vaildData();
        bool syncData();
        bool undistortedPointCloud();
        void publishData();

        rclcpp::Node::SharedPtr node_ptr_;
        std::shared_ptr<localization::ImuSubscriber> subImu_;
        std::shared_ptr<localization::LidarSubscriber> subLidar_;
        std::shared_ptr<localization::GnssSubscriber> subGnss_;

        std::deque<GnssData> unsync_gnss_buff_;
        std::deque<ImuData> unsync_imu_buff_;
        std::deque<sensor_msgs::msg::PointCloud2> unsync_cloud_buff_;

        GnssData sync_gnss_;
        ImuData sync_imu_;
        CloudData sync_cloud_;

        undistortedPoint uP;
        double current_cloud_time_;
    };
} // namespace localization
