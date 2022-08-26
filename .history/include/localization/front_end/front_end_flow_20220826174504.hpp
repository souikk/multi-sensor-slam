/*
 * @Author: Ke Zhang
 * @Date: 2022-08-19 16:22:17
 * @LastEditTime: 2022-08-26 17:45:04
 * @Description:
 */
#pragma once

#include "front_end/front_end_node.hpp"
#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/imu_subscriber.hpp"
#include "rclcpp/rclcpp.hpp"
namespace localization
{
    class FrontEndFlow
    {
    public:
        FrontEndFlow(rclcpp::Node::SharedPtr node_ptr);

    private:
        rclcpp::Node::SharedPtr node_ptr_;
    };
}