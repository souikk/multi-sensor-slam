/*
 * @Author: Ke Zhang
 * @Date: 2022-08-18 09:55:49
 * @LastEditTime: 2022-08-18 11:11:06
 * @Description
 */
#include "subscriber/imuSubscriber.hpp"

namespace localization
{
    ImuSubscriber::ImuSubscriber(rclcpp::Node::SharedPtr node_ptr)
    {
        node_ptr_ = node_ptr;
        subImu_ = node_ptr_->create_subscription<sensor_msgs::msg::Imu>("imu/data", rclcpp::QoS(10), std::bind(&ImuSubscriber::ImuSubscriber, this, std::placeholders::_1));
    }

    void ImuSubscriber::ImuSubscriber(sensor_msgs::msg::Imu::SharedPtr msg)
    {
    }
}