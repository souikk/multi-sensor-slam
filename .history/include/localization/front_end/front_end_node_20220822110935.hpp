/*
 * @Author: Ke Zhang
 * @Date: 2022-08-17 16:14:15
 * @LastEditTime: 2022-08-22 11:09:35
 * @Description:前端节点处理流程
 */
#pragma once

#include "global_path.h"

#include "rclcpp/rclcpp.hpp"

namespace localization
{
    class FrontEndNode : public rclcpp::Node
    {
    public:
        FrontEndNode(const rclcpp::NodeOptions &options);

    private:
    };
}
