/*
 * @Author: Ke Zhang
 * @Date: 2022-08-26 15:41:51
 * @LastEditTime: 2022-08-26 16:18:01
 * @Description:
 */
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "sensor_data/cloud_data.hpp"

namespace localization
{
    class CloudPublisher
    {
    public:
        CloudPublisher(rclcpp::Node::SharedPtr node_ptr, std::string topic_name, std::string frame_id);

    private:
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloud_;
    };
} // namespace localization
