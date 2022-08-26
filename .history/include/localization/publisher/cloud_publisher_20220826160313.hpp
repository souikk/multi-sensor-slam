/*
 * @Author: Ke Zhang
 * @Date: 2022-08-26 15:41:51
 * @LastEditTime: 2022-08-26 16:02:38
 * @Description:
 */
#pragma once

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "sensor_data/cloud_data.hpp"

namespace localization
{
    class CloudPublisher
    {
    public:
        CloudPublisher(rclcpp::Node::SharedPtr node_ptr, std::string topic_name, std::string frame_id);
    };
} // namespace localization
