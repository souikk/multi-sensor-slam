/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 15:00:46
 * @LastEditTime: 2022-08-26 15:55:09
 * @Description:
 */
#pragma once
#include <memory>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_data/gnss_data.hpp"
#include "preprocess_data/preprocess_data_node.hpp"


namespace localization
{
    class GnssSubscriber
    {
    public:
        GnssSubscriber(std::shared_ptr<PreprocessDataNode> node_ptr, std::string topic_name, size_t queue_size, int policy_history);

        void accessData(std::deque<GnssData> &gnss_buff);

    private:
        void gnssCallback(nav_msgs::msg::Odometry::SharedPtr msg);

        std::shared_ptr<PreprocessDataNode> node_ptr_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subGnss_;
        std::deque<GnssData> gnss_data_buf_;
    };
} // namespace localization
