/*
 * @Author: Ke Zhang
 * @Date: 2022-08-18 09:55:49
 * @LastEditTime: 2022-08-19 16:52:57
 * @Description
 */
#include "subscriber/imuSubscriber.hpp"

namespace localization
{
    ImuSubscriber::ImuSubscriber(rclcpp::Node::SharedPtr node_ptr)
    {
        node_ptr_ = std::make_shared<rclcpp::Node>(node_ptr);
        subImu_ = node_ptr_->create_subscription<sensor_msgs::msg::Imu>("imu/data", rclcpp::QoS(10), std::bind(&ImuSubscriber::imuCallback, this, std::placeholders::_1));
    }

    void ImuSubscriber::imuCallback(sensor_msgs::msg::Imu::SharedPtr msg)
    {
    }
}