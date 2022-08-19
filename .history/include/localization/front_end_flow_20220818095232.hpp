/*
 * @Author: Ke Zhang
 * @Date: 2022-08-17 16:14:15
 * @LastEditTime: 2022-08-18 09:52:16
 * @Description:前端节点处理流程
 */
#pragma once
#include "global_path.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
namespace localization
{
    class FrontEndFlow : public rclcpp::Node
    {
    public:
        FrontEndFlow();

    private:
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
        
        };
}
