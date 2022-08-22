/*
 * @Author: Ke Zhang
 * @Date: 2022-08-17 17:42:37
 * @LastEditTime: 2022-08-22 14:41:01
 * @Description: 订阅imu数据类
 */
#pragma once

#include <memory>
#include <yaml-cpp/yaml.h>

#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"

#include "global_path.h"

namespace localization
{
    class ImuSubscriber
    {
    public:
        ImuSubscriber(rclcpp::Node::SharedPtr node_ptr, std::string topic_name, int queue_size);

    private:
        void imuCallback(sensor_msgs::msg::Imu::SharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu_;
        rclcpp::Node::SharedPtr node_ptr_;
    };
}
