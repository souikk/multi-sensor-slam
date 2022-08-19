/*
 * @Author: Ke Zhang
 * @Date: 2022-08-17 16:14:15
 * @LastEditTime: 2022-08-18 11:17:44
 * @Description:前端节点处理流程
 */
#pragma once

#include "global_path.h"
#include "subscriber/imuSubscriber.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace localization
{
    class FrontEndFlow : public rclcpp::Node
    {
    public:
        FrontEndFlow();

    private:
        std::shared_ptr<rclcpp::Node::SharedPtr> subImu_;
    };
}
