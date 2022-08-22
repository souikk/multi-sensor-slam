/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 15:00:46
 * @LastEditTime: 2022-08-22 15:13:52
 * @Description:
 */
#pragma once

#include "rclcpp/rclcpp.hpp"

namespace localization
{
    class gnssSubscriber
    {
    public:
        gnssSubscriber(rclcpp::Node::SharedPtr node_ptr, std::string topic_name, int queue_size);

    private:
        void gnssCallback();
        rclcpp::Node::SharedPtr node_ptr_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subGnss_;
    }
} // namespace localization
