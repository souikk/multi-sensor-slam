/*
 * @Author: Ke Zhang
 * @Date: 2022-08-19 16:54:38
 * @LastEditTime: 2022-08-19 17:08:28
 * @Description:
 */
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace localization
{
    class LidarSubscriber
    {
    public:
        LidarSubscriber(rclcpp::Node::SharedPtr node_ptr);

    private:
        lidarCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg);
        rclcpp::Node::SharedPtr node_ptr_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2> subLidar_;
    };
}