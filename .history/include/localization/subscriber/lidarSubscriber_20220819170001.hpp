/*
 * @Author: Ke Zhang
 * @Date: 2022-08-19 16:54:38
 * @LastEditTime: 2022-08-19 16:59:38
 * @Description:
 */
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

namespace localization
{
    class LidarSubscriber
    {
        LidarSubscriber(rclcpp::Node::SharedPtr node_ptr);
    };
}