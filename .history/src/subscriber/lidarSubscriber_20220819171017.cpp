/*
 * @Author: Ke Zhang
 * @Date: 2022-08-19 17:01:35
 * @LastEditTime: 2022-08-19 17:10:16
 * @Description:
 */
#include "subscriber/lidarSubscriber.hpp"

namespace localization
{
    LidarSubscriber::LidarSubscriber(rclcpp::Node::SharedPtr node_ptr)
    {
        node_ptr_ = std::make_shared<rclcpp::Node>(node_ptr);
        subLidar_ = node_ptr_->create_subscription<sensor_msgs::msg::PointCloud2>(
            "hesai/pandar_points", 10, std::bind(&LidarSubscriber::lidarCallback(), this, std::placeholders::_1));
    }
}