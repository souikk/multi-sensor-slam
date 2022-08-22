/*
 * @Author: Ke Zhang
 * @Date: 2022-08-19 17:01:35
 * @LastEditTime: 2022-08-22 14:19:07
 * @Description:
 */

#include "subscriber/lidar_subscriber.hpp"
#include "rclcpp/rclcpp.hpp"

namespace localization
{
    LidarSubscriber::LidarSubscriber(rclcpp::Node::SharedPtr node_ptr)
    {
        node_ptr_ = node_ptr;
        std::string config_file_path = WORKSPACE_PATH + "/config/front_end/config.yaml";
        YAML::Node config = YAML::Load(config_file_path);

        subLidar_ = node_ptr->create_subscription<sensor_msgs::msg::PointCloud2>(
            "hesai/pandar_points", 10, std::bind(&LidarSubscriber::lidarCallback, this, std::placeholders::_1));
    }

    void LidarSubscriber::lidarCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "subscribe lidar message");
    }
}