/*
 * @Author: Ke Zhang
 * @Date: 2022-08-17 17:42:37
 * @LastEditTime: 2022-08-29 15:23:23
 * @Description: 订阅imu数据类
 */
#pragma once

#include <memory>
#include <deque>

#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"

#include "global_path.h"

#include "sensor_data/imu_data.hpp"

#include "preprocess_data/preprocess_data_node.hpp"

namespace localization
{
    class ImuSubscriber
    {
    public:
        ImuSubscriber(std::shared_ptr<p> node_ptr, std::string topic_name, size_t queue_size, int policy_history);

        void accessData(std::deque<ImuData> &imu_buff);

    private:
        void imuCallback(sensor_msgs::msg::Imu::SharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu_;
        rclcpp::Node::SharedPtr node_ptr_;
        std::deque<ImuData> imu_data_buf_;
    };
}
