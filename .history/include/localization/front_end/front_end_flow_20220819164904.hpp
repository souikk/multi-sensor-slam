/*
 * @Author: Ke Zhang
 * @Date: 2022-08-19 16:22:17
 * @LastEditTime: 2022-08-19 16:49:04
 * @Description:
 */
#pragma once

#include "front_end/front_end_node.hpp"
#include "rclcpp/rclcpp.hpp"
class FrontEndFlow
{
public:
    FrontEndFlow(rclcpp::Node::SharedPtr node_ptr);

private:
    std::shared_ptr<rclcpp::Node> node_ptr_;
    std::shared_ptr<localization::ImuSubscriber> subImu_;
};