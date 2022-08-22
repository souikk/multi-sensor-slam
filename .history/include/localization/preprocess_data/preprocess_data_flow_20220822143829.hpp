/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 09:58:44
 * @LastEditTime: 2022-08-22 14:38:29
 * @Description:数据预处理工作流程类的头文件
 */
#pragma once

#include "rclcpp/rclcpp.hpp"

#include "preprocess_data/preprocess_data_node.hpp"
#include "subscriber/lidar_subscriber.hpp"
#include "subscriber/imu_subscriber.hpp"

namespace localization
{
    class PreprocessDataFlow
    {
    public:
        PreprocessDataFlow(rclcpp::Node::SharedPtr node_ptr);

    private:
        InitSubscriber(rclcpp::Node::SharedPtr node_ptr, YAML::Node &config);

        rclcpp::Node::SharedPtr node_ptr_;
        std::shared_ptr<localization::ImuSubscriber> subImu_;
        std::shared_ptr<localization::LidarSubscriber> subLidar_;
    };
} // namespace localization
