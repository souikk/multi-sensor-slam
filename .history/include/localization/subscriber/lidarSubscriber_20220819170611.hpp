/*
 * @Author: Ke Zhang
 * @Date: 2022-08-19 16:54:38
 * @LastEditTime: 2022-08-19 17:06:10
 * @Description:
 */
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

namespace localization
{
    class LidarSubscriber
    {
    public:
        LidarSubscriber(rclcpp::Node::SharedPtr node_ptr);

    private:
        rclcpp::Node::SharedPtr node_ptr_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2> subLidar_;
    };
}