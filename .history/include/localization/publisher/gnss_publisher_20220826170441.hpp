/*
 * @Author: Ke Zhang
 * @Date: 2022-08-26 16:53:01
 * @LastEditTime: 2022-08-26 17:04:41
 * @Description:
 */
#pragma once
#include "rclcpp/rclcpp.hpp"

namespace localization
{
    class GnssPublisher
    {
    public:
        GnssPublisher(rclcpp::Node::SharedPtr node_ptr, std::string topic_name, std::string frame_id, int queue_size, int policy_history);

    private:
        rclcpp::Node::SharedPtr node_ptr_;
        std::string frame_id_;
    };
} // namespace localization
