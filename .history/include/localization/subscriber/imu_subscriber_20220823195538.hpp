/*
 * @Author: Ke Zhang
 * @Date: 2022-08-17 17:42:37
 * @LastEditTime: 2022-08-23 19:55:38
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
        ImuSubscriber(rclcpp::Node::SharedPtr node_ptr, std::string topic_name, size_t queue_size, int policy_history);

    private:
        void imuCallback(sensor_msgs::msg::Imu::SharedPtr msg);
        void accessData(std::deque<ImuData> &imu_buff);
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu_;
        rclcpp::Node::SharedPtr node_ptr_;
        std::deque<ImuData> imu_data_buf_;
    };
}