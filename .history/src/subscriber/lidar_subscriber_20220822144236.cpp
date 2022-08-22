/*
 * @Author: Ke Zhang
 * @Date: 2022-08-19 17:01:35
 * @LastEditTime: 2022-08-22 14:42:24
 * @Description:
 */

#include "subscriber/lidar_subscriber.hpp"

namespace localization
{
    LidarSubscriber::LidarSubscriber(rclcpp::Node::SharedPtr node_ptr, std::string topic_name, int queue_size)
    {
        node_ptr_ = node_ptr;

        subLidar_ = node_ptr_->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name, 10, std::bind(&LidarSubscriber::lidarCallback, this, std::placeholders::_1));
    }

    void LidarSubscriber::lidarCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "subscribe lidar message");
    }
}