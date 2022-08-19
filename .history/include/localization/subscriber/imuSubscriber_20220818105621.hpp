/*
 * @Author: Ke Zhang
 * @Date: 2022-08-17 17:42:37
 * @LastEditTime: 2022-08-18 10:56:21
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
        ImuSubscriber(rclcpp::Node node);

    private:
        void imuCallback(sensor_msgs::msg::Imu::SharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu_;
        rclcpp::Node node_;
    };
}
