/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 09:58:44
 * @LastEditTime: 2022-08-22 18:26:25
 * @Description:数据预处理工作流程类的头文件
 */
#pragma once

#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"

#include "preprocess_data/preprocess_data_node.hpp"
#include "subscriber/lidar_subscriber.hpp"
#include "subscriber/imu_subscriber.hpp"
#include "global_path.h"
namespace localization
{
    class PreprocessDataFlow
    {
    public:
        PreprocessDataFlow(rclcpp::Node::SharedPtr node_ptr);

    private:
        void InitSubscriber(rclcpp::Node::SharedPtr node_ptr, YAML::Node &config);

        rclcpp::Node::SharedPtr node_ptr_;
        std::shared_ptr<localization::ImuSubscriber> subImu_;
        std::shared_ptr<localization::LidarSubscriber> subLidar_;
        std::shared_ptr<localization::gnssSubscriber> subGnss_;
    };
} // namespace localization
