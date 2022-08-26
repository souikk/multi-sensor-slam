/*
 * @Author: Ke Zhang
 * @Date: 2022-08-26 16:53:01
 * @LastEditTime: 2022-08-26 16:57:30
 * @Description:
 */
#pragma once
#include "rclcpp/rclcpp.hpp"

namespace localization
{
    class GnssPublisher
    {
        GnssPublisher(rclcpp::Node::SharedPtr node_ptr, std::string topic_name, std::string frame_id, int queue_size, int policy_history);
    };
} // namespace localization
