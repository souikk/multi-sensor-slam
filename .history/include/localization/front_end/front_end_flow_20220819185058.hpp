/*
 * @Author: Ke Zhang
 * @Date: 2022-08-19 16:22:17
 * @LastEditTime: 2022-08-19 18:50:58
 * @Description:
 */
#pragma once

#include "front_end/front_end_node.hpp"
#include "subscriber/lidarSubscriber.hpp"
#include "subscriber/imuSubscriber.hpp"
#include "rclcpp/rclcpp.hpp"
namespace localization
{
    class FrontEndFlow
    {
    public:
        FrontEndFlow(rclcpp::Node::SharedPtr node_ptr);

    private:
        rclcpp::Node::SharedPtr node_ptr_;
        std::shared_ptr<localization::ImuSubscriber> subImu_;
        std::shared_ptr<localization::LidarSubscriber> subLidar;
    };
}