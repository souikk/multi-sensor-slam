/*
 * @Author: Ke Zhang
 * @Date: 2022-08-17 17:42:37
 * @LastEditTime: 2022-08-23 10:23:24
 * @Description: 订阅imu数据类
 */
#pragma once

#include <memory>
#include <deque>

#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"

#include "global_path.h"

#include "sensor_data/imu_data.hpp"
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
        std::deque<ImuData> imu_data_buf_;
    };
}
