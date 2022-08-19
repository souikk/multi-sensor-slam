/*
 * @Author: Ke Zhang
 * @Date: 2022-08-17 17:42:37
 * @LastEditTime: 2022-08-17 18:01:46
 * @Description: 订阅imu数据类
 */
#pragma once

#include "sensor_msgs/msg/imu.hpp"
#include ""
namespace localization
{
    class ImuSubscriber
    {
    public:
        ImuSubscriber();

    private:
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    }
}
