/*
 * @Author: Ke Zhang
 * @Date: 2022-08-19 16:54:38
 * @LastEditTime: 2022-08-23 10:52:12
 * @Description: 订阅lidar数据类
 */
#pragma once

#include <memory>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace localization
{
    class LidarSubscriber
    {
    public:
        LidarSubscriber(rclcpp::Node::SharedPtr node_ptr, std::string topic_name, size_t queue_size);

    private:
        void lidarCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg);
        rclcpp::Node::SharedPtr node_ptr_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLidar_;
    };
}