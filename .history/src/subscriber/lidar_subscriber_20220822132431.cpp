/*
 * @Author: Ke Zhang
 * @Date: 2022-08-19 17:01:35
 * @LastEditTime: 2022-08-22 13:24:31
 * @Description:
 */
#include "subscriber/lidar_subscriber.hpp"

namespace localization
{
    LidarSubscriber::LidarSubscriber(rclcpp::Node::SharedPtr node_ptr)
    {
        node_ptr_ = node_ptr;
        YAML::Node config
            subLidar_ = node_ptr_->create_subscription<sensor_msgs::msg::PointCloud2>(
                "hesai/pandar_points", 10, std::bind(&LidarSubscriber::lidarCallback, this, std::placeholders::_1));
    }

    void LidarSubscriber::lidarCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "subscribe lidar message");
    }
}