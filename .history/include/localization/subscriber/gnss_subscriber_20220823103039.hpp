/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 15:00:46
 * @LastEditTime: 2022-08-23 10:30:38
 * @Description:
 */
#pragma once
#include <memory>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_data/gnss_data.hpp"
namespace localization
{
    class GnssSubscriber
    {
    public:
        GnssSubscriber(rclcpp::Node::SharedPtr node_ptr, std::string topic_name, int queue_size);

    private:
        void gnssCallback(nav_msgs::msg::Odometry::SharedPtr msg);
        rclcpp::Node::SharedPtr node_ptr_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subGnss_;
        std::deque<>
    };
} // namespace localization
