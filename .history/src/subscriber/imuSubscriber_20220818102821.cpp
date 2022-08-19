/*
 * @Author: Ke Zhang
 * @Date: 2022-08-18 09:55:49
 * @LastEditTime: 2022-08-18 10:27:43
 * @Description
 */
#include "subscriber/imuSubscriber.hpp"

namespace localization
{
    ImuSubscriber::ImuSubscriber(rclcpp::Node::SharedPtr node_ptr)
    {
        node_ptr_ = std::make_shared<rclcpp::Node>(*node_ptr);
        subImu_ = node_ptr_->create_subscription<sensor_msgs::msg::Imu>();
    }
}