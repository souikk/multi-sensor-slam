/*
 * @Author: Ke Zhang
 * @Date: 2022-08-17 17:42:37
 * @LastEditTime: 2022-08-18 10:04:37
 * @Description: 订阅imu数据类
 */
#pragma once

#include <memory>

#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"

namespace localization
{
    class ImuSubscriber
    {
    public:
        ImuSubscriber(rclcpp::Node::SharedPtr noder);

    private:
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
        rclcpp::Node::SharedPtr node_pt;
    };
}
