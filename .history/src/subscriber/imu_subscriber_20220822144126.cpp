/*
 * @Author: Ke Zhang
 * @Date: 2022-08-18 09:55:49
 * @LastEditTime: 2022-08-22 14:41:25
 * @Description
 */
#include "subscriber/imu_subscriber.hpp"

namespace localization
{
    ImuSubscriber::ImuSubscriber(rclcpp::Node::SharedPtr node_ptr, std::string topic_name, int queue_size)
    {
        node_ptr_ = node_ptr;
        subImu_ = node_ptr_->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", rclcpp::QoS(10), std::bind(&ImuSubscriber::imuCallback, this, std::placeholders::_1));
    }

    void ImuSubscriber::imuCallback(sensor_msgs::msg::Imu::SharedPtr msg)
    {
    }
}