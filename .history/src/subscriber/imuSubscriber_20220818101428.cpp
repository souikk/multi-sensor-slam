/*
 * @Author: Ke Zhang
 * @Date: 2022-08-18 09:55:49
 * @LastEditTime: 2022-08-18 10:14:27
 * @Description
 */
#include "subscriber/imuSubscriber.hpp"

namespace localization
{
    ImuSubscriber::ImuSubscriber(rclcpp::Node::SharedPtr node_ptr)
    {
        node_ptr_ = std::make_shared<rclcpp::Node>(*node_ptr);
    }
}