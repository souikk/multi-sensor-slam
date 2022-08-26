/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 09:58:44
 * @LastEditTime: 2022-08-23 19:48:32
 * @Description:数据预处理工作流程类的头文件
 */
#pragma once

#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"

#include "preprocess_data/preprocess_data_node.hpp"
#include "subscriber/lidar_subscriber.hpp"
#include "subscriber/imu_subscriber.hpp"
#include "subscriber/gnss_subscriber.hpp"
#include "global_path.h"
namespace localization
{
    class PreprocessDataFlow
    {
    public:
        PreprocessDataFlow(rclcpp::Node::SharedPtr node_ptr);

        void run();

    private:
        void initSubscriber(rclcpp::Node::SharedPtr node_ptr, const YAML::Node &config);
        bool read();

        rclcpp::Node::SharedPtr node_ptr_;
        std::shared_ptr<localization::ImuSubscriber> subImu_;
        std::shared_ptr<localization::LidarSubscriber> subLidar_;
        std::shared_ptr<localization::GnssSubscriber> subGnss_;

        std::deque<GnssData> gnss_buff_;
        std::deque<ImuData> imu_buff_;
        std::deque<sensor_msgs::msg::PointCloud2> cloud_buff_;
    };
} // namespace localization
