/*
 * @Author: Ke Zhang
 * @Date: 2022-08-19 16:22:17
 * @LastEditTime: 2022-08-19 16:46:55
 * @Description:
 */
#pragma once

#include "front_end/front_end_node.hpp"
#include "rclcpp/rclcpp.hpp"
class FrontEndFlow
{
public:
    FrontEndFlow(std::shared_ptr<rclcpp::Node> node_ptr);

private:
    std::shared_ptr<rclcpp::Node> node_ptr_;
};