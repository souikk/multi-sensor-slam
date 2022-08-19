/*
 * @Author: Ke Zhang
 * @Date: 2022-08-17 16:14:15
 * @LastEditTime: 2022-08-19 16:19:49
 * @Description:前端节点处理流程
 */
#pragma once

#include "global_path.h"
#include "subscriber/imuSubscriber.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace localization
{
    class FrontEndNode : public rclcpp::Node
    {
    public:
        FrontEndNode();

    private:
        std::shared_ptr<localization::ImuSubscriber> subImu_;
    };
}
