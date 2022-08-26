/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 15:00:46
 * @LastEditTime: 2022-08-23 20:10:43
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
        GnssSubscriber(rclcpp::Node::SharedPtr node_ptr, std::string topic_name, size_t queue_size, int policy_history);

    private:
        void gnssCallback(nav_msgs::msg::Odometry::SharedPtr msg);
        void accessData(std::deque<GnssData> gnss_buff);
        rclcpp::Node::SharedPtr node_ptr_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subGnss_;
        std::deque<GnssData> gnss_data_buf_;
    };
} // namespace localization
