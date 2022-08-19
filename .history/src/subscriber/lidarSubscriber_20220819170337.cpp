/*
 * @Author: Ke Zhang
 * @Date: 2022-08-19 17:01:35
 * @LastEditTime: 2022-08-19 17:03:37
 * @Description:
 */
#include "subscriber/lidarSubscriber.hpp"

namespace localization
{
    LidarSubscriber::LidarSubscriber(rclcpp::Node::SharedPtr node_ptr)
    {
        node_ptr_ = std::make_shared<rclcpp::Node>(node_ptr);
    }
}