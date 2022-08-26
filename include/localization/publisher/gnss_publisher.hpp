/*
 * @Author: Ke Zhang
 * @Date: 2022-08-26 16:53:01
 * @LastEditTime: 2022-08-26 17:13:00
 * @Description:
 */
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "sensor_data/gnss_data.hpp"
namespace localization
{
    class GnssPublisher
    {
    public:
        GnssPublisher(rclcpp::Node::SharedPtr node_ptr, std::string topic_name, std::string frame_id, int queue_size, int policy_history);

        void publishMsg(GnssData gnss_data);

    private:
        rclcpp::Node::SharedPtr node_ptr_;
        std::string frame_id_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubGnss_;
    };
} // namespace localization
