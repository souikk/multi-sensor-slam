/*
 * @Author: Ke Zhang
 * @Date: 2022-08-18 09:55:49
 * @LastEditTime: 2022-08-18 10:57:55
 * @Description
 */
#include "subscriber/imuSubscriber.hpp"

namespace localization
{
    ImuSubscriber::ImuSubscriber(rclcpp::Node &node)
    {
        node_ptr_ = std::make_shared<rclcpp::Node>(*node_ptr);
        subImu_ = node_.create_subscription<sensor_msgs::msg::Imu>("imu/data", Qos(10), std::bind(&ImuSubscriber::ImuSubscriber, this, _1));
    }

    void ImuSubscriber::ImuSubscriber(sensor_msgs::msg::Imu::SharedPtr msg)
    {
    }
}