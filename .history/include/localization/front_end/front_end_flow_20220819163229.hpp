/*
 * @Author: Ke Zhang
 * @Date: 2022-08-19 16:22:17
 * @LastEditTime: 2022-08-19 16:32:29
 * @Description:
 */
#pragma once

#include "front_end/front_end_node.hpp"
#include "rclcpp/rclcpp.hpp"
class FrontEndFlow
{
public:
    FrontEndFlow();

private:
    std::shared_ptr<localization::FrontEndNode> node_ptr_;
};